// This file features modifications from Patrick Pfreundschuh 
// (patripfr@ethz.ch) from 2024, mainly concerning addition of photometric
// terms, clean-up, and logging, as described in: https://arxiv.org/abs/2310.01235


// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <visualization_msgs/Marker.h>

#include "feature_manager.h"
#include "image_processing.h"
#include "imu_processing.h"
#include "preprocess.h"
#include "projector.h"
#include "timing.h"
#include "use_ikfom.h"

#define INIT_TIME (0.1)
#define DIM_STATE (23)

int add_point_size = 0, kdtree_delete_counter = 0;
bool pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

double DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;

string root_dir = ROOT_DIR;
string lid_topic, imu_topic;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, FOV_DEG = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0;
int feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, scan_body_pub_en = false;
double geo_std;

vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

M4D T_IL(Eye4d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;

vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

shared_ptr<ImageProcessor> image_processor;
shared_ptr<Projector> projector;
shared_ptr<FeatureManager> feature_manager;

double photo_scale;
Eigen::MatrixXd h_geo_last;
LidarFrame current_frame;

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->normal_z;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || 
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
                need_move = true;
        }
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, 
        double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
        time_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    const std::lock_guard<std::mutex> lock(mtx_buffer);
    
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
}

double timediff_lidar_wrt_imu = 0.0;

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    {
        const std::lock_guard<std::mutex> lock(mtx_buffer);
        if (timestamp < last_timestamp_imu)
        {
            ROS_WARN("imu loop back, clear buffer");
            imu_buffer.clear();
        }
        last_timestamp_imu = timestamp;
        imu_buffer.push_back(msg);
    }
}

bool sync_packages(MeasureGroup &meas)
{

    const std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        meas.lidar_end_time = meas.lidar_beg_time + meas.lidar->header.stamp * 1.e-9;
        lidar_end_time = meas.lidar_end_time;
        if (meas.lidar->points.size() <= 1) // time too little
        {
            ROS_WARN("Too few input point cloud!\n");
        }

        lidar_pushed = true;
    }
    if (imu_buffer.back()->header.stamp.toSec() < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

V3D calculateContributions(const Eigen::MatrixXd& H, Eigen::MatrixXd& evec) {
    V3D contrib;
    contrib.setZero();

    for (size_t feat_idx = 0u; feat_idx < H.rows(); ++feat_idx) {
        for (size_t dir_idx = 0u; dir_idx < 3; ++dir_idx) {
            V3D feat_row = H.row(feat_idx).transpose();
            feat_row.normalize();
            V3D dir = evec.col(dir_idx);
            const float dotp = fabs(feat_row.dot(dir));
            if (dotp > 0.5) {
                contrib(dir_idx) += dotp;
            }
        }
    }
    return contrib;
}

Eigen::MatrixXd calculateEigenVectors(const Eigen::MatrixXd& H) {
    Eigen::MatrixXd HTH = H.transpose() * H; 
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3,3>> solve(
        HTH);
    Eigen::Matrix3d evec = solve.eigenvectors().real();
    return evec;
}

void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)* filter_size_map_min + 
                0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)* filter_size_map_min + 
                0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)* filter_size_map_min +
                 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && 
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && 
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_undistort);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string pcd_dir = string(ROOT_DIR) + "PCD/";
            bool directory_exists = boost::filesystem::exists(pcd_dir) && 
                boost::filesystem::is_directory(pcd_dir);
            if (!directory_exists)
            {
                boost::filesystem::create_directory(pcd_dir);
            }
            string all_points_dir(pcd_dir + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to " << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;

}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void h_share_model_geometric(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear();
    corr_normvect->clear();

    /** closest surface search and residual computation **/

    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;


        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];

        /** Find the closest surfaces in the map **/

         if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : 
                pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ekfom_data.h_x.resize(0,0);
        ekfom_data.h.resize(0);
        ROS_WARN("No Effective Points! \n");
        return;
    }

    /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);
 
    for (int i = 0; i < effct_feat_num; i++)
    {

        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRIX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRIX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), 
                VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

void h_share_model_photometric(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    if (feature_manager->features().empty()) {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Photometric Points! \n");
        return;
    }

    cv::Mat& img = current_frame.img_intensity;

    const int marg_size = feature_manager->margin();
    const int marg_col = img.cols - marg_size;
    const int marg_row = img.rows - marg_size;
    const int patch_size = feature_manager->patchSize();
    const int n_patch = patch_size * patch_size;
    const int n_features = feature_manager->features().size();
    const double min_range = feature_manager->minRange();
    const double max_range = feature_manager->maxRange();

    // Calculate required transforms
    M4D T_GI = M4D::Identity();
    T_GI.block<3,3>(0,0) = s.rot.toRotationMatrix();
    T_GI.block<3,1>(0,3) = s.pos;
    M3D R_IG = T_GI.block<3,3>(0,0).transpose();

    M4D T_GL = T_GI * T_IL;
    M4D T_LG = M4D::Identity();
    T_LG.block<3,3>(0,0) = T_GL.block<3,3>(0,0).transpose();
    T_LG.block<3,1>(0,3) = -T_GL.block<3,3>(0,0).transpose() * T_GL.block<3,1>(0,3);
        
    M3D R_LG = T_LG.block<3,3>(0,0);
    V3D p_LG = T_LG.block<3,1>(0,3);

    // Initialize containers
    ekfom_data.h_x = MatrixXd::Zero(n_features*n_patch, 12);
    ekfom_data.h = VectorXd::Zero(n_features*n_patch);
    Eigen::MatrixXd h_x_terms = MatrixXd::Zero(n_features*n_patch, 12);
    Eigen::VectorXd h_terms = VectorXd::Zero(n_features*n_patch);
    std::vector<bool> term_selected(n_features * n_patch, false); 

    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    // Compute Jacobian per tracked patch
    for (int j = 0; j < n_features; j++) {
        bool is_fov = true;
        std::vector<bool> px_selected(n_patch, false);

        // Iterate over all pixels in the patch
        for (int l = 0; l < n_patch; l++) {
            const V3D& p_feat_G = feature_manager->features()[j].p[l];
            V3D p_feat_Lk = R_LG * p_feat_G + p_LG;
            V3D p_feat_Li;
            V2D uv_i;
            int distortion_idx;
            if(!projector->projectUndistortedPoint(current_frame, p_feat_Lk, p_feat_Li, uv_i, distortion_idx, true)) {
                is_fov = false;
                break;
            };

            // Get the transform from the endtime of the frame to the time of the respective close point
            const M4D& T_Li_Lk = current_frame.T_Li_Lk_vec[current_frame.vec_idx[distortion_idx]];

            // Check if feature is in valid distance range
            if (p_feat_Li.norm() < min_range || p_feat_Li.norm() > max_range) {
                is_fov = false;
                break;
            }

            // Check if the point is in the margin
            if (!((uv_i(0) > marg_size) && (uv_i(0) < marg_col) && (uv_i(1) > marg_size) && (uv_i(1) < marg_row))) {
                is_fov = false;
                break;
            }

            if (current_frame.img_mask.ptr<uchar>(int(uv_i(1)))[int(uv_i(0))] == 0) {
                is_fov = false;
                break;
            }
            
            // Image Gradient
            double val_v_p = getSubPixelValue<float>(img, uv_i(0), uv_i(1)+1);
            double val_v_m = getSubPixelValue<float>(img, uv_i(0), uv_i(1)-1);
            double val_u_p = getSubPixelValue<float>(img, uv_i(0)+1, uv_i(1));
            double val_u_m = getSubPixelValue<float>(img, uv_i(0)-1, uv_i(1));

            double dIx = 0.5 * (val_u_p - val_u_m);
            double dIy = 0.5 * (val_v_p - val_v_m);

            Eigen::Matrix<double, 1, 2> dI_du;
            dI_du << dIx, dIy;
            Eigen::MatrixXd du_dp;
            projector->projectionJacobian(p_feat_Li, du_dp);


            // Jacobian of p_L_i with respect to the state, see equation (9) in the paper
            M3D R_Li_I = T_Li_Lk.block<3,3>(0,0) * T_IL.block<3,3>(0,0).transpose();
            
            Eigen::Matrix<double, 3, 6> dp_dtf;
            V3D p_feat_I = R_IG*(p_feat_G - T_GI.block<3,1>(0,3));
            M3D p_feat_I_x;
            p_feat_I_x << SKEW_SYM_MATRIX(p_feat_I);
            dp_dtf <<  R_Li_I*R_IG, -R_Li_I*p_feat_I_x;
            
            // Photometric error, see equation (7) in the paper
            double z_pho =  getSubPixelValue<float>(img, uv_i(0), uv_i(1)) - 
                feature_manager->features()[j].intensities[l];

            // Bookkeeping
            int row_idx = j*n_patch + l;
            h_terms(row_idx) = z_pho;
            h_x_terms.block<1, 6>(row_idx,0) = dI_du * du_dp * dp_dtf;           
            px_selected[l] = true;                  
        }

        // Only add the terms if the whole patch is in FOV
        if (is_fov) {
            for (size_t t_idx = 0u; t_idx < px_selected.size(); ++t_idx) {
                if (px_selected[t_idx]) {
                    term_selected[j*n_patch + t_idx] = true;
                }
            }
        } 
    } 

    int px_count = 0;
    for (size_t t_idx = 0u; t_idx < term_selected.size(); ++t_idx) {
        if (term_selected[t_idx]) {
            ekfom_data.h(px_count) = h_terms(t_idx);
            ekfom_data.h_x.block<1, 6>(px_count,0) = h_x_terms.block<1, 6>(t_idx,0);
            px_count += 1;
        }
    }

    ekfom_data.h.conservativeResize(px_count);
    ekfom_data.h_x.conservativeResize(px_count, 12);
}

void h_share_combined(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    // Calculate Photometric Terms
    esekfom::dyn_share_datastruct<double> ekfom_data_photo;
    timing::Timer photo_timer("update/photometric");
    h_share_model_photometric(s, ekfom_data_photo);
    photo_timer.Stop();

    // Calculate Point-to-Plane Terms
    esekfom::dyn_share_datastruct<double> ekfom_data_geo = ekfom_data;
    timing::Timer geo_timer("update/geometric");
    h_share_model_geometric(s, ekfom_data_geo);
    geo_timer.Stop();

    int n_terms = ekfom_data_photo.h.rows() + ekfom_data_geo.h.rows();

    if (ekfom_data_photo.h.rows() > 0) {
        float lambda = photo_scale;
        // Concatenate Photometric and Geometric Terms
        ekfom_data.h_x = Eigen::MatrixXd::Zero(n_terms, 12); 
        ekfom_data.h_x << ekfom_data_photo.h_x*lambda , ekfom_data_geo.h_x;
        ekfom_data.h = Eigen::VectorXd::Zero(n_terms);
        ekfom_data.h << ekfom_data_photo.h*lambda, ekfom_data_geo.h;
    } else {
        ekfom_data.h_x = ekfom_data_geo.h_x;
        ekfom_data.h = ekfom_data_geo.h;
    }

    if (ekfom_data_geo.h_x.rows() > 0) {
        h_geo_last = ekfom_data_geo.h_x.leftCols(3);
    } else {
        h_geo_last.resize(0,0);
    }

    if (n_terms == 0) {
        ekfom_data.valid = false;
    }
}

void log_fancy(double current_time_s, double length, int n_uninformative, int n_effective_points, int n_features, 
    int n_removed, int n_added, const std::string& coin) {

    std::cout<<"\033[2J\033[1;1H"; //clear screen
    std::cout<<"\033[33m" << coin <<std::endl; 
    std::cout<<"\033[0m"; 

    std::time_t current_time = current_time_s;
    double elapsed_time = current_time_s - first_lidar_time;
    std::string asc_time = std::asctime(std::localtime(&current_time)); asc_time.pop_back();
    std::cout << "| " << std::left << asc_time;
    std::cout << std::right << std::setfill(' ') << std::setw(35)
    << "Elapsed Time: " + string_from_double(elapsed_time) + " seconds "
    << "|" << std::endl;

    std::cout << "|------------------------------------------------------------|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
    << "Position (x,y,z)    [m] : " + string_from_double(state_point.pos(0)) + " " 
    + string_from_double(state_point.pos(1)) + " " + string_from_double(state_point.pos(2)) 
    << "|" << std::endl;

    const Eigen::Quaterniond q(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    euler *= 180.0 / M_PI;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(60)
    << "Orientation (r,p,y) [°] : " + string_from_double(euler(0)) + " " 
    + string_from_double(euler(1)) + " " + string_from_double(euler(2)) << "|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
    << "Trajectory Length   [m] : " + string_from_double(length) << "|" << std::endl;

    std::cout << "|------------------------------------------------------------|" << std::endl;


    std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
    << "Acc. Bias (x,y,z) [m/s2]  : " + string_from_double(state_point.ba(0)) + " " 
    + string_from_double(state_point.ba(1)) + " " + string_from_double(state_point.ba(2)) << "|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(59)
    << "Gyro Bias (x,y,z) [rad/s] : " + string_from_double(state_point.bg(0)) + " " 
    + string_from_double(state_point.bg(1)) + " " + string_from_double(state_point.bg(2)) << "|" << std::endl;
   
    std::cout << "|------------------------------------------------------------|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
    << "Effective Points    [#] : " <<  std::left << std::setw(33) << n_effective_points << "|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
    << "Intensity Features  [#] : " << std::left << std::setw(6) << n_features << std::left << std::setw(13) 
    << " Added: " + std::to_string(n_added) << std::left << std::setw(14)<< " Removed: " + std::to_string(n_removed) 
    << "|" << std::endl;

    std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
    << "Uninformative Dir.  [#] : " << std::left << std::setw(33) << n_uninformative << "|" << std::endl;

    std::cout << "|------------------------------------------------------------|" << std::endl;

    double mean_s = timing::Timing::GetMeanSeconds("all");
    double min_s = timing::Timing::GetMinSeconds("all");
    double max_s = timing::Timing::GetMaxSeconds("all");
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(26)
    << "Computation Time    [s] : " << std::left << "Avg: " << string_from_double(mean_s) << std::left 
    << " Max: " << string_from_double(max_s)  << std::left << " Min: " << string_from_double(min_s) 
    << " |" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    // Avoid PCL Warnings
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    std::string animation_path = ros::package::getPath("coin_lio");
    animation_path.append("/config/coin_ascii.txt");
    std::ifstream t(animation_path);
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::vector<std::string> seglist;
    std::string current;
    std::string line;
    int count = 0 ;
    while (std::getline(buffer, line))
    {
        count++;
        current.append("                ");
        current.append(line);
        current.append("\n");
        if (count % 15 == 0) {
            seglist.push_back(current);
            current = "";
        }
    }

    bool dashboard;
    bool debug;
    double eps;
    int point_filter_num;
    int n_uninformative;

    // Common Params
    nh.param<string>("common/lid_topic",lid_topic,"/ouster/points");
    nh.param<string>("common/imu_topic", imu_topic,"/ouster/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);

    // Filter Params
    nh.param<int>("filter/max_iteration", NUM_MAX_ITERATIONS, 5);
    nh.param<int>("filter/n_uninformative", n_uninformative, 25);
    nh.param<double>("filter/eps", eps, 0.001);
    nh.param<double>("filter/geo_std", geo_std, 0.001);
    nh.param<double>("filter/photo_scale", photo_scale, 0.002);

    // Mapping Params
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<int>("mapping/point_filter_num", point_filter_num, 4);
    nh.param<double>("mapping/det_range",DET_RANGE, 300.f);
    nh.param<double>("mapping/cube_side_length",cube_len, 200);
    nh.param<double>("mapping/filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("mapping/filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("mapping/fov_degree",fov_deg, 180);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    // PCD Save Params
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);

    // Preprocess Params
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, OUSTER);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    std::vector<double> lidar_to_sensor;
    bool transform_found = nh.getParam("/lidar_to_sensor_transform", lidar_to_sensor) ||
        nh.getParam("/lidar_intrinsics/lidar_to_sensor_transform", lidar_to_sensor);
    if (!transform_found || lidar_to_sensor.size() != 16) {
        ROS_WARN("No lidar to sensor transform found, setting to default value.");
        p_pre->lidar_sensor_z_offset = 0.03618;
    } else {
        p_pre->lidar_sensor_z_offset = lidar_to_sensor[11] * 0.001;
    }

    // Publishing Params
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    nh.param<bool>("publish/dashboard", dashboard, true);
    nh.param<bool>("publish/debug", debug, false);

    projector = std::make_shared<Projector>(nh);
    feature_manager = std::make_shared<FeatureManager>(nh, projector);
    image_processor = std::make_shared<ImageProcessor>(nh, projector, feature_manager);
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    double trajectory_length = 0.0;
    V3D t_G_I = V3D::Zero();
    
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    T_IL.block<3,3>(0,0) = Lidar_R_wrt_IMU;
    T_IL.block<3,1>(0,3) = Lidar_T_wrt_IMU;
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, eps);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_combined, NUM_MAX_ITERATIONS, epsi);
    
    int iter_step = 0;
    cv::Mat colorized_feature_img;

    /*** ROS subscribe initialization ***/
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>("/path", 100000);
    
    ros::Publisher intensity_publisher = nh.advertise<sensor_msgs::Image>("/filtered_image", 2000);
    ros::Publisher feature_publisher = nh.advertise<sensor_msgs::Image>("/feature_image", 2000);
    ros::Publisher previous_feature_publisher;
    ros::Publisher predicted_feature_publisher;
    ros::Publisher feature_marker_publisher;
    if (debug) {
        previous_feature_publisher = nh.advertise<sensor_msgs::Image>("/previous_feature_image", 2000);
        predicted_feature_publisher = nh.advertise<sensor_msgs::Image>("/predicted_feature_image", 2000);
        feature_marker_publisher = nh.advertise<visualization_msgs::Marker>("/feature_markers", 2000);
    } 
    
    ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

//------------------------------------------------------------------------------------------------------
    ros::AsyncSpinner spinner(0);
    spinner.start();

    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            timing::Timer full_cycle_timer("all");

            timing::Timer pre_timer("pre");

            feats_undistort = Measures.lidar;
            std::vector<M4D> vec_T_Li_Lk;
            std::vector<int> vec_idx(Measures.lidar->points.size(),0);


            // Predict state using forward integration of IMU measurements and undistort the lidar points
            timing::Timer undistort_timer("pre/undistort");
            p_imu->Process(Measures, kf, feats_undistort, vec_T_Li_Lk, vec_idx);
            undistort_timer.Stop();
           
            if (feats_undistort->empty() || (feats_undistort == NULL) || vec_T_Li_Lk.empty())
            {
                timing::Timing::Reset();
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            // Update current frame
            std_msgs::Header header;
            header.stamp.fromSec(Measures.lidar_end_time);
            header.frame_id = "lidar";
            current_frame.points_corrected = feats_undistort;
            current_frame.T_Li_Lk_vec = vec_T_Li_Lk;
            current_frame.vec_idx = vec_idx;
            current_frame.header = header;

            // Create intensity image and apply filters
            image_processor->createImages(current_frame);
            sensor_msgs::ImagePtr img_msg =
                cv_bridge::CvImage(header, "mono8", current_frame.img_photo_u8).toImageMsg();
            intensity_publisher.publish(img_msg);
            
            if (debug && (!colorized_feature_img.empty())) {
                // Visualized feature position predicted by IMU integration
                M4D T_GI = M4D::Identity();
                T_GI.block<3,3>(0,0) = state_point.rot.toRotationMatrix();
                T_GI.block<3,1>(0,3) = state_point.pos;
                M3D R_IG = T_GI.block<3,3>(0,0).transpose();

                M4D T_GL = T_GI * T_IL;
                M4D T_LG = M4D::Identity();
                T_LG.block<3,3>(0,0) = T_GL.block<3,3>(0,0).transpose();
                T_LG.block<3,1>(0,3) = -T_GL.block<3,3>(0,0).transpose() * T_GL.block<3,1>(0,3);

                M3D R_LG = T_LG.block<3,3>(0,0);
                V3D p_LG = T_LG.block<3,1>(0,3);
                cv::Mat predicted_feature_img;
                cv::cvtColor(current_frame.img_photo_u8, predicted_feature_img, CV_GRAY2RGB);
                int patch_size = feature_manager->patchSize();
                int center_idx = int(patch_size * patch_size) / 2; 
                for (auto feature : feature_manager->features()) {
                    const V3D& p_feat_G = feature.p[center_idx];
                    V3D p_feat_Lk = R_LG * p_feat_G + p_LG;
                    V3D p_feat_Li;
                    V2D uv_i;
                    int distortion_idx;
                    if(!projector->projectUndistortedPoint(current_frame, p_feat_Lk, p_feat_Li, uv_i, distortion_idx, 
                        true)) {
                        continue;
                    };
                    cv::circle(predicted_feature_img, cv::Point2f(uv_i(0), uv_i(1)), 5, cv::Scalar(0, 185, 255.0), 2);
                }

                sensor_msgs::ImagePtr feat_pred_msg =
                    cv_bridge::CvImage(current_frame.header, "bgr8", predicted_feature_img).toImageMsg();
                predicted_feature_publisher.publish(feat_pred_msg);

                sensor_msgs::ImagePtr feat_prev_msg =
                    cv_bridge::CvImage(current_frame.header, "bgr8", colorized_feature_img).toImageMsg();
                previous_feature_publisher.publish(feat_prev_msg);
            }


            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
            // Segment the map in lidar FOV 
            lasermap_fov_segment();

            // Downsample points by skipping
            PointCloudXYZI::Ptr feats_undistort_skip(new PointCloudXYZI());
            feats_undistort_skip->reserve(feats_undistort->points.size()/point_filter_num);
            for (size_t idx = 0u; idx < feats_undistort->points.size(); idx += point_filter_num) {
                feats_undistort_skip->points.push_back(feats_undistort->points[idx]);
            }

            // Downsample points using voxel filter
            downSizeFilterSurf.setInputCloud(feats_undistort_skip);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();
            
            pre_timer.Stop();

            // Initialize the map kdtree
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }

            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);

            // Iterated State Update       
            timing::Timer update_timer("update");
            double solve_H_time;
            kf.update_iterated_dyn_share_modified(geo_std, solve_H_time);
            update_timer.Stop();

            timing::Timer feature_timer("features");
            // Identify uninformative directions
            std::vector<V3D> weak_directions_g;
            if (h_geo_last.rows() > 3) {
                auto eig_vec = calculateEigenVectors(h_geo_last);
                V3D contribs_pos = calculateContributions(h_geo_last, eig_vec);

                for (int idx = 0; idx < 3; ++idx) {
                    if (contribs_pos(idx) < n_uninformative) {
                        weak_directions_g.push_back(eig_vec.col(idx));
                    }
                }
            }
            
            state_point = kf.get_x();
            M4D T_GI = M4D::Identity();
            T_GI.block<3,3>(0,0) = state_point.rot.toRotationMatrix();
            T_GI.block<3,1>(0,3) = state_point.pos;

            // Rotate uninformative directions to lidar frame
            std::vector<V3D> weak_directions_l;
            const M4D T_GL = T_GI * T_IL;
            for (const auto& vec : weak_directions_g) {
                weak_directions_l.push_back(T_GL.block<3,3>(0,0).transpose()*vec);
            }

            // If no uninformative directions are found, use the default ones
            if (weak_directions_l.empty()) {
                weak_directions_l.push_back(V3D(1,0,0));
                weak_directions_l.push_back(V3D(0,1,0));
                weak_directions_l.push_back(V3D(0,0,1));
            }

            // Track features with updated state and initialize new ones
            feature_manager->updateFeatures(current_frame, weak_directions_l, T_GL);
            feature_timer.Stop();


            // Publish feature image
            cv::cvtColor(current_frame.img_photo_u8, colorized_feature_img, CV_GRAY2RGB);
            for (int j = 0; j < feature_manager->features().size(); j++) {
                const V2D& uv = feature_manager->features()[j].center;
                cv::circle(colorized_feature_img, cv::Point2f(uv(0), uv(1)), 5, cv::Scalar(71, 215, 253), 2);
            }
            sensor_msgs::ImagePtr feat_img_msg =
                cv_bridge::CvImage(current_frame.header, "bgr8", colorized_feature_img).toImageMsg();
            feature_publisher.publish(feat_img_msg);

            if (debug) {
                visualization_msgs::Marker marker;
                int patch_size = feature_manager->patchSize();
                int center_idx = int(patch_size * patch_size) / 2; 
                marker.header.frame_id = "camera_init";
                marker.header.stamp = current_frame.header.stamp;
                marker.ns = "feature";
                marker.pose.orientation.x = 0;
                marker.pose.orientation.y = 0;
                marker.pose.orientation.z = 0;
                marker.pose.orientation.w = 1;
                marker.type = visualization_msgs::Marker::LINE_LIST;
                marker.action = visualization_msgs::Marker::ADD;
                marker.scale.x = 0.01;
                marker.scale.x = 0.01;
                marker.color.a = 0.25;
                marker.color.r = 253./255.;
                marker.color.g = 215./255.;
                marker.color.b = 71./255.;
                geometry_msgs::Point p_imu;
                p_imu.x = state_point.pos(0);
                p_imu.y = state_point.pos(1);
                p_imu.z = state_point.pos(2);

                for (int j = 0; j < feature_manager->features().size(); j++) {
                    const V3D& p_G = feature_manager->features()[j].p[center_idx];
                    geometry_msgs::Point p_feat;
                    p_feat.x = p_G(0);
                    p_feat.y = p_G(1);
                    p_feat.z = p_G(2);
                    marker.points.push_back(p_imu);
                    marker.points.push_back(p_feat);
                }
                feature_marker_publisher.publish(marker);
            }

            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            // Publish odometry 
            publish_odometry(pubOdomAftMapped);

            // Add points to map kdtree
            map_incremental();

            // Publish points
            if (path_en) publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);

            full_cycle_timer.Stop();

            if (dashboard) {
                double segment_length = (T_GI.block<3,1>(0,3) - t_G_I).norm();
                t_G_I = T_GI.block<3,1>(0,3);
                trajectory_length += segment_length;
                log_fancy(Measures.lidar_end_time, trajectory_length, weak_directions_g.size(),h_geo_last.rows(), 
                    feature_manager->features().size(), feature_manager->nAdded(), feature_manager->nRemoved(), 
                    seglist[iter_step % seglist.size()]);
            }
            iter_step++;
        } 
        else {
            sleep(0.0005);
        }
        status = ros::ok();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string pcd_dir = string(ROOT_DIR) + "PCD/";
        bool directory_exists = boost::filesystem::exists(pcd_dir) && 
            boost::filesystem::is_directory(pcd_dir);
        if (!directory_exists)
        {
            boost::filesystem::create_directory(pcd_dir);
        }
        string all_points_dir(pcd_dir + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to " << all_points_dir <<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    return 0;
}
