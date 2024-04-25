// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#ifndef COIN_LIO_FEATURE_MANAGER_H_
#define COIN_LIO_FEATURE_MANAGER_H_

#include <string>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>
#include "projector.h"


#include <common_lib.h>


struct Feature {
        int life_time;  // num of tracking
        V2D center;  // Center Pixel coordinate in last frame
        std::vector<double> intensities; // Pixel intensities
        std::vector<V3D> p; // 3D position in global frame
        std::vector<V2D> uv; // Pixel coordinates in last frame
};

class FeatureManager {
 public:
    FeatureManager(ros::NodeHandle& nh, std::shared_ptr<Projector> projector);
    
    void updateFeatures(const LidarFrame& frame, const std::vector<V3D>& V, const M4D& T_GL);

    const std::vector<Feature>& features() const { return features_; }

    const int patchSize() const { return patch_size_; }

    const double minRange() const { return min_range_; }

    const double maxRange() const { return max_range_; }

    const int margin() const { return marg_size_; }

    const int nRemoved() const { return n_removed_last_; }

    const int nAdded() const { return n_added_last_; }

 private:
    void loadParameters(ros::NodeHandle& n);
    
    void detectFeatures(const LidarFrame& frame, const std::vector<V3D>& V, const M4D& T_GL);
    
    void detectFeaturesComp( const LidarFrame& frame, const std::vector<V3D>& V, const int n_features, 
      std::vector<cv::Point>& features_uv,  std::vector<V3D>& features_p) const;
    
    void detectFeaturesStrong( const LidarFrame& frame, const int n_features, 
      std::vector<cv::Point>& features_uv,  std::vector<V3D>& features_p) const;
    
    void detectFeaturesRandom( const LidarFrame& frame, const int n_features, 
      std::vector<cv::Point>& features_uv,  std::vector<V3D>& features_p) const;

    void trackFeatures(const LidarFrame& frame, const M4D& T_GL);

    void removeFeatures(const std::vector<int>& idx);

    void updateMask();

    std::vector<string> feature_modes = {"comp", "strongest", "random"};
    std::shared_ptr<Projector> projector_;
    ros::Publisher moment_pub;
    std::vector<Feature> features_;
    Eigen::MatrixXi patch_idx_;
    cv::Mat img_prev_;
    cv::Mat mask_;
    cv::Mat mask_margin_;

    double min_range_;
    double max_range_;
    double thr_gradmin_;
    double ncc_threshold_;
    double range_threshold_;
    int suppression_radius_;
    int patch_size_;
    int rows_, cols_;
    int num_features_;
    int max_lifetime_;
    int marg_size_;
    int n_removed_last_;
    int n_added_last_;
    string mode_;
    bool debug_;

    bool IsFov(const cv::Point2f& pt) const;
 };

 #endif // COIN_LIO_FEATURE_MANAGER_H_