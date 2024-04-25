// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#ifndef COIN_LIO_PROJECTOR_H_
#define COIN_LIO_PROJECTOR_H_

#include <common_lib.h>
#include "ros/ros.h"

class Projector {
  public:
    Projector(ros::NodeHandle nh);

    void createImages(LidarFrame& frame) const;
    void projectionJacobian(const V3D& p, Eigen::MatrixXd& du_dp) const;
    bool projectUndistortedPoint(const LidarFrame& frame, const V3D& p_L_k, V3D& p_L_i, V2D& uv, int& distortion_idx, 
      bool round = false) const;
    size_t indexFromPixel(const V2D& px) const;
    bool projectPoint(const V3D& point, V2D& uv) const;
    bool isFOV(const V2D& uv) const;
    int rows() const {return rows_;}
    int cols() const {return cols_;}


  private:
    void loadParameters(ros::NodeHandle nh);
    size_t vectorIndexFromRowCol(const size_t row, const size_t col) const;
    double beam_offset_m_;
    int u_shift_;
    size_t rows_;
    size_t cols_;
    std::vector<int> offset_lut_;
    std::vector<double> elevation_angles_;
    std::vector<int> idx_to_v_;
    std::vector<int> idx_to_u_;
    M3D K_;
};

#endif  // COIN_LIO_PROJECTOR_H_