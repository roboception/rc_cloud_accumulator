#define PCL_NO_PRECOMPILE
#include "cloud_accumulator.h"
#include <limits>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>

namespace rc
{

/****************************** Utility functions ********************************************/

inline geometry_msgs::TransformStamped myPoseStampedMsgToTF(const geometry_msgs::PoseStamped & msg)
{
  geometry_msgs::TransformStamped bt;
  bt.transform.rotation = msg.pose.orientation;
  bt.transform.translation.x = msg.pose.position.x;
  bt.transform.translation.y = msg.pose.position.y;
  bt.transform.translation.z = msg.pose.position.z;
  bt.header.stamp = msg.header.stamp;
  bt.header.frame_id = msg.header.frame_id;
  bt.child_frame_id = "camera";
  return bt;
}


inline Eigen::Affine3f toAffine(const geometry_msgs::TransformStamped& tf_stamped)
{
  const geometry_msgs::Vector3& t = tf_stamped.transform.translation;//just abbreviate
  const geometry_msgs::Quaternion& q = tf_stamped.transform.rotation;//just abbreviate

  Eigen::Affine3f result = Eigen::Affine3f::Identity();
  result.translation() << t.x, t.y, t.z;
  Eigen::Quaternionf rot(q.w, q.x, q.y, q.z);
  result.rotate(rot);
  return result;
}


///Filter if size >= 0, otherwise just copy the poninter
void voxelGridFilter(double size, pointcloud_t::Ptr input, pointcloud_t::Ptr& output)
{
  if(size <= 0.0){ output = input; }
  else
  {
    pcl::VoxelGrid<point_t> vgf;
    vgf.setInputCloud(input);
    vgf.setLeafSize(size, size, size);
    vgf.filter(*output);
  }
}

/****************************** CloudAccumulator ********************************************/

CloudAccumulator::CloudAccumulator(double voxelgrid_size_live,
                                   double voxelgrid_size_final,
                                   double min_distance,
                                   double max_distance,
                                   std::string output_filename,
                                   bool live_only,
                                   bool start_paused)
  : tf_buffer_(ros::Duration(std::numeric_limits<int>::max(), 0)),
    display_cloud_(new pointcloud_t()),
    voxelgrid_size_live_(voxelgrid_size_live),
    voxelgrid_size_final_(voxelgrid_size_final),
    min_distance_(min_distance),
    max_distance_(max_distance),
    output_filename_(output_filename),
    live_only_(live_only),
    pause_(start_paused),
    viewer_thread_(boost::ref(viewer_))
{
  viewer_.addCloudToViewer(display_cloud_);//so update can be called
}


void CloudAccumulator::pointCloudCallback(const pointcloud_t::ConstPtr& pointcloud)
{
  if(pause_){ return; }
  static int counter = 1;
  ROS_INFO_THROTTLE(10, "Received %d point cloud%s", counter, counter == 1 ? "" : "s");
  ++counter;
  double timestamp = pointcloud->header.stamp * 1e-6;//pcl stamps are in milliseconds
  ROS_DEBUG("Storing cloud (%.3f)", timestamp);

  //Create the filtering object to create along the optical axis (Z)
  pointcloud_t::Ptr distance_filtered_cloud(new pointcloud_t());
  pcl::PassThrough<point_t> pass;
  pass.setInputCloud(pointcloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_distance_, max_distance_);
  pass.filter(*distance_filtered_cloud);
  if(!live_only_) { cloud_map_[timestamp] = distance_filtered_cloud; }//store for later corrections

  const Eigen::Affine3f transformation = lookupTransform(timestamp);
  if(transformation.matrix()(0,0) == transformation.matrix()(0,0)) //not nan
  {
    pointcloud_t::Ptr transformed_cloud(new pointcloud_t());
    pcl::transformPointCloud(*distance_filtered_cloud, *transformed_cloud, transformation);

    boost::mutex::scoped_lock guard(display_cloud_mutex_);
    *transformed_cloud += *display_cloud_;
    //Voxelgridfilter to speedup display
    voxelGridFilter(voxelgrid_size_live_, transformed_cloud, display_cloud_);
    viewer_.updateCloudInViewer(display_cloud_);
  }
  else
  {
    ROS_INFO_THROTTLE(5, "Cannot display point cloud, the (live) camera pose is not available. "
                      "Is Stereo INS or SLAM running?");
    ROS_DEBUG("No (live) camera pose available for cloud at %.3f.", timestamp);
  }
  ROS_DEBUG("Done Storing cloud (%.3f)", timestamp);
}


void CloudAccumulator::trajectoryCallback(const nav_msgs::PathConstPtr& path)
{
  ROS_INFO("Received Trajectory with %zu poses", path->poses.size());
  boost::mutex::scoped_lock guard(tf_buffer_mutex_);
  tf_buffer_.clear();

  for(size_t i = 0; i < path->poses.size(); ++i)
  //for(const geometry_msgs::PoseStamped& pose : path->poses)
  {
    tf_buffer_.setTransform(myPoseStampedMsgToTF(path->poses[i]), "trajectory");
  }
}


void CloudAccumulator::poseCallback(const geometry_msgs::PoseStampedConstPtr& current_pose)
{
  if(pause_){ return; }
  ROS_INFO_ONCE("Received first live pose");
  boost::mutex::scoped_lock guard(tf_buffer_mutex_);
  tf_buffer_.setTransform(myPoseStampedMsgToTF(*current_pose), "current_pose");
}


Eigen::Affine3f CloudAccumulator::lookupTransform(double timestamp)
{
  ros::Time rosstamp; rosstamp.fromSec(timestamp);//pcl timestamp is in microsec
  boost::mutex::scoped_lock guard(tf_buffer_mutex_);

  if(tf_buffer_.canTransform("world", "camera", rosstamp)) try
  {
    geometry_msgs::TransformStamped pose;
    Eigen::Affine3f result = toAffine(tf_buffer_.lookupTransform("world", "camera", rosstamp)); //target, source
    return result;
  }
  catch(tf2::LookupException ex) { ROS_WARN("%s", ex.what()); }
  catch(tf2::ExtrapolationException ex) {  ROS_WARN("%s", ex.what()); }

  Eigen::Affine3f invalid_result;
  invalid_result.matrix().setConstant(std::numeric_limits<double>::quiet_NaN());
  return invalid_result;
}


pointcloud_t::Ptr CloudAccumulator::rebuildCloud()
{
  ROS_INFO("(Re-)merging all stored clouds with latest poses.");
  int counter = 0;
  pointcloud_t::Ptr merged(new pointcloud_t());
  for(std::map<double, pointcloud_t::ConstPtr>::iterator it = cloud_map_.begin(); it != cloud_map_.end(); ++it)
  {
    const Eigen::Affine3f transformation = lookupTransform(it->first);
    if(transformation.matrix()(0,0) == transformation.matrix()(0,0)) //not nan
    {
      ++counter;
      pointcloud_t transformed_cloud;
      pcl::transformPointCloud(*(it->second), transformed_cloud, transformation);
      *merged += transformed_cloud;
      if(counter % 10 == 0){ ROS_INFO("Merged %d of %zu clouds.", counter, cloud_map_.size()); }
    }
    else { ROS_DEBUG("Skipped cloud at %.3f in merge, cannot get its position", it->first); }
  }
  ROS_INFO("Merged %d/%zu clouds.", counter, cloud_map_.size());

  pointcloud_t::Ptr voxel_filtered(new pointcloud_t());
  voxelGridFilter(voxelgrid_size_final_, merged, voxel_filtered);
  return voxel_filtered;
}


bool CloudAccumulator::saveCloud(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp)
{
  if(!live_only_) // consider slam corrections
  {
    pointcloud_t::Ptr merged = rebuildCloud();
    pcl::io::savePCDFileASCII(output_filename_, *merged);

    //Now show the final cloud
    //(it will quickly get downfiltered in the point cloud callback though if not paused)
    boost::mutex::scoped_lock guard(display_cloud_mutex_);
    display_cloud_ = merged;
    viewer_.addCloudToViewer(display_cloud_);
  }
  else { pcl::io::savePCDFileASCII(output_filename_, *display_cloud_); }//just save what we have

  ROS_INFO("Saved point cloud to %s", output_filename_.c_str());
  return true;
}


bool CloudAccumulator::togglePause(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp)
{
  pause_ = !pause_;
  return true;
}

}//namespace rc
