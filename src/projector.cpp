// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#include "projector.h"
#include <stdexcept>

#define DUPLICATE_POINTS 10

Projector::Projector(ros::NodeHandle nh) {
    try {
        loadParameters(nh);
    } catch (const std::runtime_error& e) {
        ROS_ERROR_STREAM(e.what());
        exit(1);
    }

    for (auto& angle : elevation_angles_) {
        angle *= M_PI/180.;
    }

    double fy = - static_cast<double>(rows_) / fabs(elevation_angles_[0] - 
        elevation_angles_[elevation_angles_.size() - 1]);
    double fx = - static_cast<double>(cols_) / (2 * M_PI);
    double cy = rows_ / 2;
    double cx = cols_ / 2;

    K_ << fx, 0, cx,
          0, fy, cy,
          0, 0, 1;

    // mm to m
    beam_offset_m_ *= 1e-3;
        
    // bookkeeping for lookup from idx to row and column
    idx_to_v_ = std::vector<int>(rows_*cols_, 0);
    idx_to_u_ = std::vector<int>(rows_*cols_, 0);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            auto idx = indexFromPixel(V2D(i,j));
            idx_to_v_[idx] = i;
            idx_to_u_[idx] = j - u_shift_;
            if (idx_to_u_[idx] < 0) {
             idx_to_u_[idx] += cols_;
            }
            if (idx_to_u_[idx] >= cols_) {
             idx_to_u_[idx] -= cols_;
            }
        }
    }
};

void Projector::loadParameters(ros::NodeHandle nh) {
    float rows;
    if (!(nh.getParam("/data_format/pixels_per_column", rows) || 
          nh.getParam("/lidar_data_format/pixels_per_column", rows))) {
        throw std::runtime_error("Missing rows parameter");
        return;
    }
    rows_ = static_cast<size_t>(rows);

    float cols;
    if (!(nh.getParam("/data_format/columns_per_frame", cols) ||
          nh.getParam("/lidar_data_format/columns_per_frame", cols))) {
        throw std::runtime_error("Missing cols parameter");
        return;
    }
    cols_ = static_cast<size_t>(cols);

    if (!(nh.getParam("/lidar_origin_to_beam_origin_mm", beam_offset_m_) || 
          nh.getParam("/beam_intrinsics/lidar_origin_to_beam_origin_mm", beam_offset_m_))) {
        throw std::runtime_error("Missing beam_offset_m parameter");
        return;
    }

    if (!(nh.getParam("/data_format/pixel_shift_by_row", offset_lut_) || 
          nh.getParam("/lidar_data_format/pixel_shift_by_row", offset_lut_))) {
        throw std::runtime_error("Missing offset parameter");
        return;
    }

    if (!(nh.getParam("/beam_altitude_angles", elevation_angles_) || 
          nh.getParam("/beam_intrinsics/beam_altitude_angles", elevation_angles_))) {
        throw std::runtime_error("Missing alt parameter");
        return;
    }

    float u_shift;
    if (!nh.getParam("image/u_shift", u_shift)) {
        throw std::runtime_error("Missing column shift parameter");
        return;
    }
    u_shift_ = static_cast<int>(u_shift);
}

size_t Projector::vectorIndexFromRowCol(const size_t row, const size_t col) const {
    return (row * cols_ + col) * DUPLICATE_POINTS;
}

void Projector::createImages(LidarFrame& frame) const {
    frame.img_intensity = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    frame.img_range = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    frame.img_idx = cv::Mat::ones(rows_, cols_, CV_32SC1) * (-1);
    frame.proj_idx = std::vector<int>(rows_*cols_*DUPLICATE_POINTS, 0);
    
    std::vector<int> idx_to_vk(frame.points_corrected->points.size(), -1);
    std::vector<int> idx_to_uk(frame.points_corrected->points.size(), -1);

    // Create a projected image from the undistorted point cloud, dubbed "undistortion map" in the paper
    #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
    #endif
    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j) {
        const V3D p_Lk = frame.points_corrected->points[j].getVector3fMap().cast<double>();
        V2D px_k;
        if (!projectPoint(p_Lk, px_k)) continue;
        idx_to_uk[j] = std::round(px_k(0));
        idx_to_vk[j] = std::round(px_k(1));
    }


    #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
    #endif
    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j) {
        // Use lookup for dense image
        int v_i = idx_to_v_[frame.points_corrected->points[j].normal_x];
        int u_i = idx_to_u_[frame.points_corrected->points[j].normal_x];
        const float p_norm = frame.points_corrected->points[j].getVector3fMap().norm();
        frame.img_range.ptr<float>(v_i)[u_i] = frame.points_corrected->points[j].normal_y;
        frame.img_intensity.ptr<float>(v_i)[u_i] = frame.points_corrected->points[j].intensity;
        
        // to keep track of index in undistorted cloud
        frame.img_idx.ptr<int>(v_i)[u_i] = j;
    }

    for (size_t j = 0; j < frame.points_corrected->points.size(); ++j) {
        int u_k = idx_to_uk[j];
        int v_k = idx_to_vk[j];
        if (u_k < 0 || v_k < 0) continue;
        size_t start_idx = vectorIndexFromRowCol(v_k, u_k);
        size_t offset = frame.proj_idx[start_idx] + 1;
        if (offset >= DUPLICATE_POINTS) continue;
        size_t idx = start_idx + offset;
        frame.proj_idx[idx] = j;      
        frame.proj_idx[start_idx] = offset;  
    }
}

size_t Projector::indexFromPixel(const V2D& px) const {
    const int vv = (int(px(1)) + cols_ - offset_lut_[int(px(0))]) % cols_;
    const size_t index = px(0)* cols_ + vv;
    return index;
};

bool Projector::projectPoint(const V3D& point, V2D& uv) const {
    // Spherical projection model that accounts for the beam offset, as described in the paper figure 2
    const double L = sqrt(point.x()*point.x() + point.y()*point.y()) - beam_offset_m_;
    const double R = sqrt(point.z()*point.z() + L*L);
    const double phi = atan2(point.y(), point.x());
    const double theta = asin(point.z()/R);
    uv.x() = K_(0, 0) * phi + K_(0, 2);

    // Instead of directly using theta, we use the lookup table to find the corresponding row and interpolate 
    // for subpixel accuracy

    if (theta > elevation_angles_[0]) {
        uv.y() = 0;
        return false;
    } else if (theta < elevation_angles_[rows_ - 1]) {
        uv.y() = rows_ - 1;
        return false;
    }

    // Angle above
    auto greater = (std::upper_bound(elevation_angles_.rbegin(), elevation_angles_.rend(), theta) + 1).base();
    // Angle below
    auto smaller = greater + 1;
    if (greater == elevation_angles_.end()) {
        uv.y() = rows_ - 1;
    } else {
        // Interpolate pixel
        uv.y() = std::distance(elevation_angles_.begin(), greater);
        uv.y() += (*greater - theta) / (*greater - *smaller);
    }       

    return isFOV(uv);
}

bool Projector::isFOV(const V2D& uv) const {
    return (uv.x() >= 0 && uv.x() <= cols_ - 1 && uv.y() >= 0 && uv.y() <= rows_ - 1);
}

void Projector::projectionJacobian(const V3D& p, Eigen::MatrixXd& du_dp) const {
    // Calculate projection jacobian with respect to 3D point position, as expressed in formula 8 in paper
    double rxy = p.head<2>().norm();
    double L = rxy - beam_offset_m_;
    double R2 = L*L + p.z()*p.z();
    double irxy = 1./rxy;
    double irxy2 = irxy * irxy;
    double fx_irxy2 = K_(0,0) * irxy2;

    du_dp = Eigen::MatrixXd::Zero(2,3);
    du_dp << -fx_irxy2 * p.y(), fx_irxy2 * p.x(), 0,
        -K_(1,1)*p.x()*p.z()/(L*R2), -K_(1,1)*p.y()*p.z()/(L*R2), K_(1,1)*L/R2;
}

bool Projector::projectUndistortedPoint(const LidarFrame& frame,const V3D& p_L_k, V3D& p_L_i, V2D& uv, 
    int& distortion_idx, bool round) const {
    // Project to image
    V2D uv_k;
    if (!projectPoint(p_L_k, uv_k)) {
        return false;
    }

    if (round) {
        uv_k(0) = std::round(uv_k(0));
        uv_k(1) = std::round(uv_k(1));
    }

    distortion_idx = -1;
    int row = uv_k(1);
    int col = uv_k(0);

    int idx = vectorIndexFromRowCol(row, col);
    // Look up the index of the undistortion transformation that belongs to this pixel
    if (frame.proj_idx[idx] == 0) {
        row = 0;
        while (row < frame.img_intensity.rows) {
            idx = vectorIndexFromRowCol(row, col);
            if (frame.proj_idx[idx] > 0) {
                break;
            } else {
                ++row;
            }
        }
    }

    if (row >= frame.img_intensity.rows) {
        return false;
    }

    // If multiple points project to pixel in the undistortion map, we select the closest one to the feature point
    if(frame.proj_idx[idx] > 1) {
        float min_dist = std::numeric_limits<float>::max();
        for (int i = 1; i <= frame.proj_idx[idx]; i++) {
            const int j = frame.proj_idx[idx + i];
            const V3D p_cand = frame.points_corrected->points[j].getVector3fMap().cast<double>();
            const V3D diff = p_L_k - p_cand;
            const float dist = diff.norm();
            if (dist < min_dist) {
                min_dist = dist;
                distortion_idx = j;
            }
        }
    } else {
        distortion_idx = frame.proj_idx[idx + 1];
    }

    if (distortion_idx < 0) {
        return false;
    }

    // Lookup the inverse (T_Li_Lk) of the undistortion transformation (T_Lk_Li)
    const M4D& T_Li_Lk = frame.T_Li_Lk_vec[frame.vec_idx[distortion_idx]];

    // Express what the feature point would have been expressed in the lidar frame at the time of the 
    // respective point, basically "distort" the point to distorted lidar frame
    p_L_i = T_Li_Lk.block<3,3>(0,0) * p_L_k + T_Li_Lk.block<3,1>(0,3);

    // Now we can project this point to the actual dense image (which is recorded in the distorted frame)
    if (!projectPoint(p_L_i, uv)) {
        return false;
    }

    return true;
}