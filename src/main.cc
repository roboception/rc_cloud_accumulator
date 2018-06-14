#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include "cloud_accumulator.h"
#include <pcl/visualization/pcl_visualizer.h>

visualizer_ptr_t createVisualizer()
{
  visualizer_ptr_t viewer(new visualizer_t("Roboception Cloud Viewer"));
  viewer->setBackgroundColor(0.1, 0.1, 0.1);//dark gray
  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();
  viewer->setCameraPosition(-1,0,0, 0,0,0, 0,0,1);
  return viewer;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rc_cloud_accumulator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  double downsampling_size_live = pnh.param("voxel_grid_size_live", 0.05);//0.0 or less: Off
  double downsampling_size_final = pnh.param("voxel_grid_size_final", 0.01);//0.0 or less: Off
  double min_distance = pnh.param("minimum_distance", 0.1);
  double max_distance = pnh.param("maximum_distance", 5.0);
  std::string output_filename = pnh.param("output_filename", std::string("cloud.pcd"));
  bool start_paused = pnh.param("start_paused", false);
  bool live_only = pnh.param("live_only", false);

  visualizer_ptr_t viewer = createVisualizer();

  rc::CloudAccumulator acc(viewer,
                           downsampling_size_live, downsampling_size_final,
                           min_distance, max_distance, output_filename,
                           live_only, start_paused);

  ros::Subscriber traj_sub;
  if(!live_only)//there would be no corrections otherwise
  {
    traj_sub = nh.subscribe<nav_msgs::Path>("trajectory", 2,
                                            &rc::CloudAccumulator::trajectoryCallback, &acc);
  }

  ros::Subscriber cloud_sub =
    nh.subscribe<pointcloud_t>("/stereo/points2", 1, &rc::CloudAccumulator::pointCloudCallback, &acc);

  ros::Subscriber pose_sub =
    nh.subscribe<geometry_msgs::PoseStamped>("pose", 50, &rc::CloudAccumulator::poseCallback, &acc);

  ros::ServiceServer pause_srv =
    nh.advertiseService("toggle_pause", &rc::CloudAccumulator::togglePause, &acc);

  ros::ServiceServer merge_srv =
    nh.advertiseService("save_cloud", &rc::CloudAccumulator::saveFinalCloud, &acc);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::Rate r(25);
  while(ros::ok())
  {
    if(!viewer->wasStopped()) { viewer->spinOnce(10); }
    r.sleep();
  }
  ros::waitForShutdown();
}
