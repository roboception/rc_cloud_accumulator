#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include "cloud_accumulator.h"
#include <pcl/visualization/pcl_visualizer.h>

visualizer_ptr_t createVisualizer()
{
  visualizer_ptr_t viewer(new visualizer_t("Roboception Cloud Viewer"));
  viewer->setBackgroundColor(0.15, 0.15, 0.15);
  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();
  viewer->setCameraPosition(-1,0,0, 0,0,0, 0,0,1);
  return viewer;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_accumulator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  double downsampling_live = pnh.param("voxel_grid_size_live", 0.05);//0.0 or less: Off
  double downsampling_merged = pnh.param("voxel_grid_size_merged", 0.01);//0.0 or less: Off
  double min_distance = pnh.param("minimum_distance", 0.1);
  double max_distance = pnh.param("maximum_distance", 5.0);
  bool start_paused = pnh.param("start_paused", false);

  visualizer_ptr_t viewer = createVisualizer();

  rc::CloudAccumulator acc(viewer,
                           downsampling_live, downsampling_merged,
                           min_distance, max_distance,
                           start_paused);

  ros::Subscriber traj_sub =
    nh.subscribe<nav_msgs::Path>("trajectory", 2, &rc::CloudAccumulator::trajectoryCallback, &acc);

  ros::Subscriber cloud_sub =
    nh.subscribe<pointcloud_t>("/stereo/points2", 10, &rc::CloudAccumulator::pointCloudCallback, &acc);

  ros::Subscriber pose_sub =
    nh.subscribe<geometry_msgs::PoseStamped>("pose", 10, &rc::CloudAccumulator::poseCallback, &acc);

  ros::ServiceServer pause_srv =
    nh.advertiseService("toggle_pause", &rc::CloudAccumulator::togglePause, &acc);

  ros::ServiceServer merge_srv =
    nh.advertiseService("merge_clouds", &rc::CloudAccumulator::mergeClouds, &acc);

  ros::Rate r(25);
  while(ros::ok())
  {
    ros::spinOnce();
    if(!viewer->wasStopped()) { viewer->spinOnce(100); }
    r.sleep();
  }
}
