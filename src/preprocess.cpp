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

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  avia_handler(msg,pcl_out);
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


void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  uint plsize = msg->point_num;
  pcl_out->clear();
  pcl_out->reserve(plsize);

  for(uint i=1; i<plsize; i++)
  {
      if((abs(msg->points[i].x - msg->points[i-1].x) < 1e-8)
          || (abs(msg->points[i].y - msg->points[i-1].y) < 1e-8)
          || (abs(msg->points[i].z - msg->points[i-1].z) < 1e-8)
          || (msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y < blind)
          || (msg->points[i].line > scan_line))
      {
          continue;
      }

       if (i % point_filter_num == 0){
          PointType pl_buffer;
          pl_buffer.x = msg->points[i].x;
          pl_buffer.y = msg->points[i].y;
          pl_buffer.z = msg->points[i].z;
          pl_buffer.intensity = msg->points[i].reflectivity;
          pl_buffer.curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points
          pcl_out->points.push_back(pl_buffer);
      }
  }
}
