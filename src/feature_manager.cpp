// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#include "feature_manager.h"

FeatureManager::FeatureManager(ros::NodeHandle& nh, std::shared_ptr<Projector> projector) : projector_(projector) {
    loadParameters(nh);

    rows_ = projector_->rows();
    cols_ = projector_->cols();

    // Set margin mask to keep a certain distance from the image border
    mask_margin_ = cv::Mat::zeros(rows_, cols_, CV_8U);
    cv::Rect roi_margin(marg_size_, marg_size_,
        cols_-2*marg_size_, rows_-2*marg_size_);
    mask_margin_(roi_margin) = 255;

    patch_idx_ = Eigen::MatrixXi::Zero(patch_size_*patch_size_, 2); 
    int off = int(patch_size_/2.);
     for (int i = -off; i<off+1; i++)
    {
        for (int j = -off; j<off+1; j++)
        {
            patch_idx_((i+off)*(patch_size_)+j+off,0) = i;
            patch_idx_((i+off)*(patch_size_)+j+off,1) = j;
        }
    }

    if (debug_) {
        moment_pub = nh.advertise<sensor_msgs::Image>("moment_image", 2000);
    }
}

void FeatureManager::loadParameters(ros::NodeHandle& nh) {
    nh.param<bool>("publish/debug", debug_, false);
    nh.param<int>("image/margin", marg_size_, 10);
    nh.param<double>("image/min_range", min_range_, 0.3);
    nh.param<double>("image/max_range", max_range_, 20);
    nh.param<double>("image/grad_min", thr_gradmin_, 20);
    nh.param<double>("image/ncc_threshold", ncc_threshold_, 0.3);
    nh.param<double>("image/range_threshold", range_threshold_, 0.2);
    nh.param<int>("image/suppression_radius", suppression_radius_, 10);
    nh.param<int>("image/patch_size", patch_size_, 5);
    nh.param<int>("image/num_features", num_features_, 65);
    nh.param<int>("image/max_lifetime", max_lifetime_, 30);
    nh.param<string>("image/selection_mode", mode_, "comp");
    if (std::find(feature_modes.begin(), feature_modes.end(), mode_) == feature_modes.end()) {
        ROS_ERROR("Invalid mode, setting to complementary");
        mode_ = "comp";
    }
    if (patch_size_ % 2 == 0) {
        ROS_ERROR("Patch size must be odd, setting to 5");
        patch_size_ = 5;
    }
}

void FeatureManager::updateFeatures(const LidarFrame& frame, const std::vector<V3D>& V, const M4D& T_GL) {
    trackFeatures(frame, T_GL);
    updateMask();
    detectFeatures(frame, V, T_GL);
}

void FeatureManager::detectFeatures(const LidarFrame& frame, const std::vector<V3D>& V, const M4D& T_GL) {

    // Amount of new features to add
    int n_features = num_features_ - features_.size();

    if (n_features <= 0) {
        return;
    }

    std::vector<cv::Point> features_uv;
    std::vector<V3D> features_p;

    
    // Detect features according to selection mode
    if (mode_=="comp") { 
        detectFeaturesComp(frame, V, n_features, features_uv, features_p);
    } else if (mode_=="strongest") {
        detectFeaturesStrong(frame, n_features, features_uv, features_p);
    } else if (mode_=="random") {
        detectFeaturesRandom(frame, n_features, features_uv, features_p);
    }
    else {
        ROS_ERROR("Invalid mode");
        return;
    }

    features_.reserve(num_features_);

    for (size_t i = 0; i < features_uv.size(); i++) {
        Feature feature;

        feature.life_time = 1;
        feature.center << features_uv[i].x, features_uv[i].y;
            
        // Fill in points and intensities
        for (size_t l = 0; l < patch_idx_.rows(); l++) {
            const int point_idx = frame.img_idx.ptr<int>(features_uv[i].y + patch_idx_(l,1))[features_uv[i].x + 
                patch_idx_(l,0)];
                
            // Point in lidar frame
            V3D p_l = frame.points_corrected->points[point_idx].getVector3fMap().cast<double>();
            // Point in global frame
            V3D p_g = T_GL.block<3,3>(0,0)*p_l + T_GL.block<3,1>(0,3);
            feature.p.push_back(p_g);
            feature.intensities.push_back(frame.img_intensity.ptr<float>(int(features_uv[i].y+patch_idx_(l,1))) [
                int(features_uv[i].x+patch_idx_(l,0))]);
            // Pixel coordinates in last frame
            feature.uv.push_back(V2D(features_uv[i].x+patch_idx_(l,0),features_uv[i].y+patch_idx_(l,1)));
        }
        features_.push_back(feature);
    }   
    img_prev_ = frame.img_photo_u8.clone();
    n_added_last_ = features_uv.size();
}

void FeatureManager::detectFeaturesComp(const LidarFrame& frame, const std::vector<V3D>& V, const int n_features,
    std::vector<cv::Point>& features_uv, std::vector<V3D>& features_p) const {

    // Compute gradient image
    cv::Mat dx_abs, dy_abs, grad_img;
    cv::convertScaleAbs(frame.img_dx, dx_abs);
    cv::convertScaleAbs(frame.img_dy, dy_abs);
    cv::addWeighted(dx_abs, 0.5, dy_abs, 0.5, 0, grad_img);

    // Mask out pixels close to existing features and image borders
    cv::Mat valid_mask = frame.img_mask & mask_;
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (size_t v = 0; v < frame.img_mask.rows; v++) {
        for (size_t u = 0; u < frame.img_mask.cols; u++) {
            if (valid_mask.ptr<uchar>(v)[u] == 0) {
                grad_img.ptr<uchar>(v)[u] = 0;
            }
        }
    }

    // Sort pixels by gradient score
    std::vector<std::pair<double, cv::Point>> grad_scores_vec;
    grad_scores_vec.reserve(grad_img.total());
    for (size_t v = 0; v < grad_img.rows; v++) {
        for (size_t u = 0; u < grad_img.cols; u++) {
            if (grad_img.ptr<uchar>(v)[u] > thr_gradmin_) {
                grad_scores_vec.push_back(std::make_pair(grad_img.ptr<uchar>(v)[u], cv::Point(u,v)));
            }
        }
    }
    std::sort( grad_scores_vec.begin(), grad_scores_vec.end(),
        [&]( const std::pair<double, cv::Point>& lhs, const std::pair<double, cv::Point>& rhs )
        {
            return lhs.first > rhs.first;
        } );

    // Non-maximum Suppression    
    cv::Mat feature_mask = valid_mask.clone();
    std::vector<cv::Point> candidates;
    for (auto& score_pair : grad_scores_vec) {
        if (feature_mask.ptr<uchar>(score_pair.second.y)[score_pair.second.x] == 0) continue;
        candidates.push_back(score_pair.second);
        cv::circle(feature_mask, score_pair.second, suppression_radius_, 0, -1);
    }

    // Vector to store scores for each direction
    std::vector<std::vector<std::pair<double, int>>> scores_vec(V.size(), std::vector<std::pair<double, int>>());
    for (size_t vec_idx = 0u; vec_idx < V.size(); vec_idx++) {
        scores_vec[vec_idx] = std::vector<std::pair<double, int>>(candidates.size(), std::make_pair(0, 0));
    }

    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (size_t i = 0; i < candidates.size(); i++) {

        // Calculate eigenvalues and eigenvectors of the image patch around feature candidate
        int offset = int(patch_size_/2)+1;
        cv::Rect roi_ij(candidates[i].x - offset, candidates[i].y - offset, patch_size_+2, patch_size_+2);
        cv::Mat img_area = frame.img_intensity(roi_ij);
        cv::Mat eivecim;
        cv::cornerEigenValsAndVecs(img_area, eivecim, 5, 3);
        float eig_val_1 = eivecim.ptr<cv::Vec6f>(offset)[offset][0];
        float eig_val_2 = eivecim.ptr<cv::Vec6f>(offset)[offset][1];

        float i_x, i_y;
        // Eigenvalues come unsorted so we need to figure it out ourselves
        if (eig_val_1 >= eig_val_2) {
            i_x = eivecim.ptr<cv::Vec6f>(offset)[offset][2];
            i_y = eivecim.ptr<cv::Vec6f>(offset)[offset][3];
        } else {
            i_x = eivecim.ptr<cv::Vec6f>(offset)[offset][4];
            i_y = eivecim.ptr<cv::Vec6f>(offset)[offset][5];
        }

        // Approximate gradient of the image patch
        Eigen::Matrix<double, 1, 2> dI_du;
        dI_du << i_x, i_y;

        const int point_idx = frame.img_idx.ptr<int>(candidates[i].y)[candidates[i].x];
        if (point_idx < 0) {
            continue;
        };

        V3D p = frame.points_corrected->points[
            point_idx].getVector3fMap().cast<double>();

        Eigen::MatrixXd du_dp;
        projector_->projectionJacobian(p, du_dp);

        // Calculate score for each direction as shown in formula 4 in paper
        for (size_t vec_idx = 0u; vec_idx < V.size(); vec_idx++) {              
            Eigen::Matrix<double, 2, 1> dpi = du_dp * V[vec_idx];
            dpi.normalize();
            float c_i = fabs(dI_du * dpi);
            scores_vec[vec_idx][i] = std::make_pair(c_i, i);
        }
    }

    // Sort scores for each direction
    for (auto& score_vec : scores_vec) {
        std::sort( score_vec.begin(), score_vec.end(),
                        [&]( const std::pair<double, int>& lhs, const std::pair<double, int>& rhs ) {
                        return lhs.first > rhs.first;
                        } );
    }

    cv::Mat moment_col;
    cv::cvtColor(frame.img_photo_u8, moment_col, CV_GRAY2RGB);

    // Select best features for each direction
    std::vector<int> sorted_indices;

    for (size_t idx = 0u; idx < scores_vec[0].size(); ++idx) {
        // Feature candidates that are sorted for each direction with ascending score
        for (size_t vec_idx = 0u; vec_idx < V.size(); vec_idx++) {
            int i = scores_vec[vec_idx][idx].second;
            // Check if feature is already added from another direction
            auto it = std::find(sorted_indices.begin(), sorted_indices.end(), i);
            if (it != sorted_indices.end()) {
                continue;
            }
            sorted_indices.push_back(i);
            // Colorize feature candidates based on score
            float score = scores_vec[vec_idx][idx].first;
            cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(score*60, 255, 255));
            cv::Mat bgr;
            cv::cvtColor(hsv, bgr, CV_HSV2BGR);
            cv::Scalar color = cv::Scalar((int)bgr.at<cv::Vec3b>(0, 0)[0],(int)bgr.at<cv::Vec3b>(0, 0)[1],
                (int)bgr.at<cv::Vec3b>(0, 0)[2]);
            cv::circle(moment_col, candidates[i], 4, color, 2);
        }
    }

    features_p.reserve(n_features);
    features_uv.reserve(n_features);
    for (size_t idx = 0; idx < sorted_indices.size(); idx++) {
        int i = sorted_indices[idx];
        features_uv.push_back(candidates[i]);
        int point_idx = frame.img_idx.ptr<int>(candidates[i].y)[candidates[i].x];
        const auto p = frame.points_corrected->points[point_idx].getVector3fMap().cast<double>();
        features_p.push_back(p);

        if (features_uv.size() >= n_features) {
            break;
        }
    }

    if (debug_) {
        sensor_msgs::ImagePtr moment_img =
        cv_bridge::CvImage(frame.header, "bgr8", moment_col).toImageMsg();
        moment_pub.publish(moment_img);
    }
}


void FeatureManager::detectFeaturesStrong(const LidarFrame& frame, const int n_features,
    std::vector<cv::Point>& features_uv, std::vector<V3D>& features_p) const {

    // Compute gradient image
    cv::Mat dx_abs, dy_abs, grad_img;
    cv::convertScaleAbs(frame.img_dx, dx_abs);
    cv::convertScaleAbs(frame.img_dy, dy_abs);
    cv::addWeighted(dx_abs, 0.5, dy_abs, 0.5, 0, grad_img);

    // Mask out pixels close to existing features and image borders
    cv::Mat valid_mask = frame.img_mask & mask_;
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int v = 0; v < frame.img_mask.rows; v++) {
        for (int u = 0; u < frame.img_mask.cols; u++) {
            if (valid_mask.ptr<uchar>(v)[u] == 0) {
                grad_img.ptr<uchar>(v)[u] = 0;
            }
        }
    }

    // Sort pixels by gradient score
    std::vector<std::pair<double, cv::Point>> grad_scores_vec;
    grad_scores_vec.reserve(grad_img.total());
    for (int v = 0; v < grad_img.rows; v++) {
        for (int u = 0; u < grad_img.cols; u++) {
            if (grad_img.ptr<uchar>(v)[u] > thr_gradmin_) {
                grad_scores_vec.push_back(std::make_pair(grad_img.ptr<uchar>(v)[u], cv::Point(u,v)));
            }
        }
    }
    std::sort( grad_scores_vec.begin(), grad_scores_vec.end(),
        [&]( const std::pair<double, cv::Point>& lhs, const std::pair<double, cv::Point>& rhs )
        {
            return lhs.first > rhs.first;
        } );

    // Non-maximum Suppression    
    cv::Mat feature_mask = valid_mask.clone();
    std::vector<cv::Point> candidates;
    for (auto& score_pair : grad_scores_vec) {
        if (feature_mask.ptr<uchar>(score_pair.second.y)[score_pair.second.x] == 0) continue;
        candidates.push_back(score_pair.second);
        cv::circle(feature_mask, score_pair.second, suppression_radius_, 0, -1);
    }

    features_p.reserve(n_features);
    features_uv.reserve(n_features);
    for (int idx = 0; idx < candidates.size(); idx++) {
        features_uv.push_back(candidates[idx]);
        int point_idx = frame.img_idx.ptr<int>(candidates[idx].y)[candidates[idx].x];
        const auto p = frame.points_corrected->points[
            point_idx].getVector3fMap().cast<double>();
        features_p.push_back(p);
        if (features_uv.size() >= n_features) {
            break;
        }
    }
}

void FeatureManager::detectFeaturesRandom(const LidarFrame& frame, const int n_features,
    std::vector<cv::Point>& features_uv, std::vector<V3D>& features_p) const {
    
    // Mask out pixels close to existing features and image borders
    cv::Mat valid_mask = frame.img_mask & mask_;

    std::vector<cv::Point> candidates;
    std::vector<cv::Point> valid_locations;
    cv::findNonZero(valid_mask, valid_locations);

    // Randomly sample valid pixels while considering suppression radius
    while (candidates.size() < n_features && valid_locations.size() > 0) {
        int idx = rand() % valid_locations.size();
        candidates.push_back(valid_locations[idx]);
        cv::circle(valid_mask, valid_locations[idx], suppression_radius_, 0, -1);
        cv::findNonZero(valid_mask, valid_locations);
    }

    features_p.reserve(n_features);
    features_uv.reserve(n_features);
    for (size_t idx = 0; idx < candidates.size(); idx++) {
        features_uv.push_back(candidates[idx]);
        int point_idx = frame.img_idx.ptr<int>(candidates[idx].y)[candidates[idx].x];
        const auto p = frame.points_corrected->points[
            point_idx].getVector3fMap().cast<double>();
        features_p.push_back(p);
        if (features_uv.size() >= n_features) {
            break;
        }
    }
}

void FeatureManager::trackFeatures(const LidarFrame& frame, const M4D& T_GL) {
    std::vector<int> remove_idx;
    M4D T_LG = M4D::Identity();
    T_LG.block<3,3>(0,0) = T_GL.block<3,3>(0,0).transpose();
    T_LG.block<3,1>(0,3) = -T_LG.block<3,3>(0,0) * T_GL.block<3,1>(0,3);
    M3D R_LG = T_LG.block<3,3>(0,0);
    V3D p_LG = T_LG.block<3,1>(0,3);

    for (size_t j = 0; j < features_.size(); j++) {
        const int n_pixel = features_[j].p.size();

        // To store intensities of current patch and reference patch
        Eigen::VectorXd I0 = Eigen::VectorXd::Zero(n_pixel);
        Eigen::VectorXd I1 = Eigen::VectorXd::Zero(n_pixel);
        double I0_mean = 0;
        double I1_mean = 0;

        std::vector<V2D> uv_patch;
        uv_patch.reserve(n_pixel);

        bool not_occluded = true;

        for (int l = 0; l < n_pixel; l++) {
            const V3D p_feat_G = features_[j].p[l];

            // Transform to lidar frame at end of frame
            const V3D p_feat_Lk = R_LG * p_feat_G + p_LG;

            V3D p_feat_Li;
            V2D uv_i;
            int distortion_idx;
            if(!projector_->projectUndistortedPoint(frame, p_feat_Lk, p_feat_Li, uv_i, distortion_idx, false)) {
                not_occluded = false;
                break;
            };

            // Check if the point is still in the image and outside the border margin
            if ((uv_i(0) < marg_size_) || (uv_i(0) > frame.img_intensity.cols - marg_size_) ||
                (uv_i(1) < marg_size_) || (uv_i(1) > frame.img_intensity.rows - marg_size_)) {
                not_occluded = false;
                break;
            }

            // Check if the point falls into a masked out area
            if (frame.img_mask.ptr<uchar>(int(uv_i(1)))[int(uv_i(0))] == 0) {
                not_occluded = false;
                break;
            }

            // Check if the point is occluded by another point or disappeared
            const double r_old = p_feat_Li.norm();
            const double r_new = frame.img_range.ptr<float>(int(uv_i(1)))[int(uv_i(0))];
            if (fabs(r_old - r_new)>range_threshold_) {
                not_occluded = false;
                break;
            }

            // Fill in intensities for NCC check
            I0(l, 0) = features_[j].intensities[l];
            I1(l, 0) = getSubPixelValue<float>(frame.img_intensity, uv_i(0), uv_i(1));
            // I0(l, 0) = getSubPixelValue<uchar>(img_prev_, features_[j].uv[l](0), features_[j].uv[l](1));
            // I1(l, 0) = getSubPixelValue<uchar>(frame.img_photo_u8, uv_i(0), uv_i(1));
            I0_mean += I0(l,0);
            I1_mean += I1(l,0);

            uv_patch.push_back(uv_i);
        }

        // Update patch
        features_[j].uv = uv_patch;

        // Calculate Normalized Cross Correlation (NCC) between the two patches
        I0_mean = I0_mean / n_pixel;
        I1_mean = I1_mean / n_pixel;
        double N0 = 0;
        double D0 = 0;
        double D1 = 0;

        for (int i = 0; i < n_pixel; i ++) {
            N0 += (I0(i,0) - I0_mean) * (I1(i,0) - I1_mean);
            D0 += (I0(i,0) - I0_mean) * (I0(i,0) - I0_mean);
            D1 += (I1(i,0) - I1_mean) * (I1(i,0) - I1_mean);
        }

        double ncc_j = N0/sqrt(D0*D1);  // compute NCC

        // Update feature if it is not occluded and NCC is above threshold and if it has not reached max lifetime
        if ((features_[j].life_time < max_lifetime_) && not_occluded && (ncc_j > ncc_threshold_)) {
            features_[j].life_time += 1;
            features_[j].center = features_[j].uv[int(n_pixel/2)];
        } else {
            remove_idx.push_back(j);
        }
    }

    // Remove features that did not pass the checks
    if (remove_idx.size() > 0) {
        removeFeatures(remove_idx);
    }
}

void FeatureManager::removeFeatures(const std::vector<int>& idx) {
    n_removed_last_ = idx.size();
    std::vector<Feature> features_remaining;
    for (size_t i = 0; i < features_.size(); i ++) {
        if (std::find(idx.begin(), idx.end(), i) == idx.end()) features_remaining.push_back(features_[i]);
    }

    features_ = features_remaining;
}

void FeatureManager::updateMask() {
    // Update mask according to feature positions to avoid features closer than suppression_radius_
    mask_ = mask_margin_.clone();
    for (auto& feature : features_) {
        cv::Point2f draw_u;
        draw_u.x = feature.center(0,0);
        draw_u.y = feature.center(1,0);
        cv::circle(mask_, draw_u, suppression_radius_, 0, -1);
    }
}

