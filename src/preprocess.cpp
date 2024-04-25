#include "preprocess.h"

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)
  {
  case OUSTER:
    ouster_handler(msg, pcl_out);
    break;

  default:
    printf("LiDAR type not supported");
    break;
  }
}

void Preprocess::ouster_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pcl_out->reserve(plsize);
  pcl::uint64_t max_time = 0;
  for (size_t i = 0; i < pl_orig.points.size(); i++)
  {

    double range = pl_orig.points[i].getVector3fMap().norm();

    if (range < blind) continue;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z - lidar_sensor_z_offset;

    if (reflectivity) {
      added_pt.intensity = pl_orig.points[i].reflectivity;
    } else {
      added_pt.intensity = pl_orig.points[i].intensity;
    }
    
    // to keep track of original point index
    added_pt.normal_x = i;
    added_pt.normal_y = range;
    added_pt.normal_z = 0;
    added_pt.curvature = pl_orig.points[i].t / 1e6; // curvature unit: ms

    pcl_out->points.push_back(added_pt);
  
    if (pl_orig.points[i].t > max_time) max_time = pl_orig.points[i].t;
  }
  pcl_out->header.stamp = max_time;
}
