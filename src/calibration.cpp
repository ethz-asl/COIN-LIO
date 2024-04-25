// Copyright (c) 2024, Patrick Pfreundschuh
// https://opensource.org/license/bsd-3-clause

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#include <thread>
#include <future>
#include <unordered_set>
#include "preprocess.h"
#include "projector.h"
#include <boost/functional/hash.hpp> 
#define foreach BOOST_FOREACH

int main(int argc, char** argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;

    std::string bag_path;
    nh.param<string>("bag_path", bag_path, "");
    std::string topic;
    nh.param<string>("topic", topic, "");
    int n_skip;
    nh.param<int>("n_skip", n_skip, 10);
    int n_total;
    nh.param<int>("n_total", n_total, 500);

    std::vector<double> lidar_to_sensor;
    bool transform_found = nh.getParam("/lidar_to_sensor_transform", lidar_to_sensor) ||
    nh.getParam("/lidar_intrinsics/lidar_to_sensor_transform", lidar_to_sensor);

    ros::param::set("image/u_shift", 0);

    shared_ptr<Preprocess> p_pre(new Preprocess());
    if (!transform_found || lidar_to_sensor.size() != 16) {
        ROS_WARN("No lidar to sensor transform found, setting to default value.");
        p_pre->lidar_sensor_z_offset = 0.03618;
    } else {
        p_pre->lidar_sensor_z_offset = lidar_to_sensor[11] * 0.001;
    }
    
    shared_ptr<Projector> projector;
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, OUSTER);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.5);
    projector = std::make_shared<Projector>(nh);


    std::cout << "Loading Bag from Path: " << bag_path << std::endl;
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::cout << "Loading Clouds from Topic: " << topic << std::endl;
    rosbag::View view(bag, rosbag::TopicQuery({topic}));

    std::cout << "Processing Clouds, this can take a while." << std::endl;

    double u_shift = 0.;
    int point_count = 0;
    int cloud_count = 0;
    int cloud_count_total = 0;
    foreach(rosbag::MessageInstance const m, view)
    {
        if (m.getTopic() != topic)
        {
            continue;
        }
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg == NULL) continue;
        cloud_count_total++;
        if (cloud_count_total % n_skip != 0) continue;
        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);   
        if (ptr->empty()) continue;
        cloud_count++;
        if (cloud_count > n_total) break;
        LidarFrame current_frame;  
        current_frame.points_corrected = ptr;
        current_frame.T_Li_Lk_vec = std::vector<M4D>(ptr->size(), M4D::Identity());
        current_frame.vec_idx = std::vector<int>(ptr->size(), 0);
        projector->createImages(current_frame);
        for (int u = 0; u < projector->cols(); u++) {
            for (int v = 0; v < projector->rows(); v++) {
                const int idx = current_frame.img_idx.ptr<int>(v)[u];
                if (idx == -1) continue;
                V2D px;
                V3D p = current_frame.points_corrected->points[idx].getVector3fMap().cast<double>();
                if (projector->projectPoint(p, px)) {
                    double diff = u - px(0);
                    if (diff > projector->cols() / 2) diff = diff - projector->cols();
                    if (diff < - projector->cols() / 2) diff = projector->cols() + diff;
                    u_shift = u_shift + (diff - u_shift) / (point_count + 1);
                    point_count++;
                }
            }
        }
    }

    std::cout << "Processed " << cloud_count << " clouds out of " << cloud_count_total << std::endl;
    std::cout << "Calculated Column Shift: " << std::round(u_shift) << std::endl;
    bag.close();

    return 0;

    
}