// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#ifndef COIN_LIO_IMAGE_PROCESSOR_H_
#define COIN_LIO_IMAGE_PROCESSOR_H_

#include <common_lib.h>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"

#include "feature_manager.h"
#include "projector.h"

class ImageProcessor {
  public:
    ImageProcessor(ros::NodeHandle nh, std::shared_ptr<Projector> projector , std::shared_ptr<FeatureManager> manager);

    void createImages(LidarFrame& frame);

  private:
    void loadParameters(ros::NodeHandle nh);
    void removeLines(cv::Mat& img);
    void filterBrightness(cv::Mat& img);
    void createMask(const cv::Mat& img, cv::Mat& mask);
    std::shared_ptr<Projector> projector_;
    cv::Mat low_pass_fir_;
    cv::Mat high_pass_fir_;
    cv::Mat kernel_dx_;
    cv::Mat kernel_dy_;
    cv::Mat kernel_erosion_;
    cv::Size window_size_;
    std::vector<cv::Rect> masks_;
    bool reflectivity_;
    bool remove_lines_;
    bool brightness_filter_;
    double intensity_scale_;
    double min_range_;
    double max_range_;
    int rows_;
    int cols_;
};

#endif  // COIN_LIO_IMAGE_PROCESSOR_H_