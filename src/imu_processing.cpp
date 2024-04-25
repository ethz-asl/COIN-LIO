#include "imu_processing.h"
#include <ros/ros.h>
#include "use_ikfom.h"

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

void ImuProcess::Reset()
{
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;

  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);

  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001;
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, 
  PointCloudXYZI &pcl_out, std::vector<M4D>& vec_T_Li_Lk,
  std::vector<int>& vec_idx)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;

  /*** store points at equal timestamps and the corresponding point indices ***/
  std::unordered_map<int, std::vector<int>> timestamp_map;
  timestamp_map.reserve(pcl_out.points.size());
  for (size_t idx = 0; idx < pcl_out.points.size(); idx++)
  {
    const int t_ns = pcl_out.points[idx].curvature * 1e6;
    timestamp_map[t_ns].push_back(idx);
  }

  /*** extract individual timestamps and sort times in ascending order ***/
  std::vector<int> timestamps;
  timestamps.reserve(timestamp_map.size());
  for (auto it = timestamp_map.begin(); it != timestamp_map.end(); it++)
  {
    timestamps.push_back(it->first);
  }
  std::sort(timestamps.begin(), timestamps.end());

  /*** initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.reserve(v_imu.size());
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, 
    imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;

    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head->header.stamp.toSec() < last_lidar_end_time_)
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, 
      imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  if (pcl_out.points.begin() == pcl_out.points.end()) return;

  /*** pose at frame-end ***/
  M4D T_Ik_G = M4D::Identity();
  T_Ik_G.block<3,3>(0,0) = imu_state.rot.toRotationMatrix().transpose();
  T_Ik_G.block<3,1>(0,3) = -T_Ik_G.block<3,3>(0,0) * imu_state.pos;
  /*** transform from Lidar to IMU ***/
  M4D T_I_L = M4D::Identity();
  T_I_L.block<3, 3>(0, 0) = imu_state.offset_R_L_I.toRotationMatrix();
  T_I_L.block<3, 1>(0, 3) = imu_state.offset_T_L_I;
  /*** transform from IMU to Lidar ***/
  M4D T_L_I = M4D::Identity();
  T_L_I.block<3,3>(0,0) = T_I_L.block<3,3>(0,0).transpose();
  T_L_I.block<3,1>(0,3) = -T_I_L.block<3,3>(0,0).transpose() * 
    T_I_L.block<3,1>(0,3);

  M4D T_G_Ii = M4D::Identity();
  vec_T_Li_Lk = std::vector<M4D>(timestamps.size(), M4D::Identity());
  std::vector<M4D> vec_T_Lk_Li(timestamps.size(), M4D::Identity());

  int t_idx = timestamps.size() - 1;

  /*** calculate T_Lk_Li at required times ***/
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    acc_imu<<VEC_FROM_ARRAY(tail->acc);
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);
    
    for(; float(timestamps[t_idx])*1.e-9 > head->offset_time; t_idx--)
    {
      dt = float(timestamps[t_idx])*1.e-9 - head->offset_time;

      /*** IMU pose at time i ***/
      T_G_Ii.block<3, 3>(0, 0) = R_imu * Exp(angvel_avr, dt);
      T_G_Ii.block<3, 1>(0, 3) = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

      /*** T_Lk_Li ***/
      vec_T_Lk_Li[t_idx] = T_L_I * (T_Ik_G * T_G_Ii * T_I_L);

      /*** T_Li_Lk ***/
      vec_T_Li_Lk[t_idx].block<3,3>(0,0) = 
          vec_T_Lk_Li[t_idx].block<3,3>(0,0).transpose();
      vec_T_Li_Lk[t_idx].block<3,1>(0,3) = 
        - vec_T_Li_Lk[t_idx].block<3,3>(0,0) * vec_T_Lk_Li[t_idx].block<3,1>(0,3);
  
      if (t_idx == 0) break;
    }
    if (t_idx == 0) break;
  }

  /*** undistort the lidar points using the respective T_LK_Li ***/
  #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
  #endif
  for (size_t idx = 0u; idx < timestamps.size(); idx++) {
    for (int p_idx : timestamp_map[timestamps[idx]]) {
      V3D P_i = pcl_out.points[p_idx].getVector3fMap().cast<double>();
      V3D P_k = (vec_T_Lk_Li[idx].block<3,3>(0,0) * P_i) + vec_T_Lk_Li[idx].block<3,1>(0,3);

      pcl_out.points[p_idx].x = P_k(0);
      pcl_out.points[p_idx].y = P_k(1);
      pcl_out.points[p_idx].z = P_k(2);
      vec_idx[p_idx] = idx;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, 
  PointCloudXYZI::Ptr cur_pcl_un_, std::vector<M4D>& vec_T_Li_Lk, std::vector<int>& vec_idx)
{
  if(meas.imu.empty()) {return;};
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;

    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
    }
    cur_pcl_un_->points.clear();
    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_, vec_T_Li_Lk, vec_idx);
}