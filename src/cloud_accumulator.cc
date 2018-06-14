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
#include <boost/lexical_cast.hpp>

namespace rc
{

/****************************** Utility functions ********************************************/

void addCloudToViewer(const pointcloud_t::Ptr pointcloud, visualizer_ptr_t& viewer)
{
  pcl::visualization::PointCloudColorHandlerRGBField<point_t> rgb(pointcloud);
  double timestamp = pointcloud->header.stamp * 1e-6;//pcl stamps are in milliseconds
  std::string id = boost::lexical_cast<std::string>(timestamp);
  viewer->addPointCloud<point_t>(pointcloud, rgb, id);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
}

void updateCloudInViewer(const pointcloud_t::Ptr pointcloud, visualizer_ptr_t& viewer)
{
  pcl::visualization::PointCloudColorHandlerRGBField<point_t> rgb(pointcloud);
  double timestamp = pointcloud->header.stamp * 1e-6;//pcl stamps are in milliseconds
  std::string id = boost::lexical_cast<std::string>(timestamp);
  viewer->updatePointCloud<point_t>(pointcloud, rgb, id);
}

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

/****************************** CloudAccumulator ********************************************/

CloudAccumulator::CloudAccumulator(visualizer_ptr_t viewer,
                                   double voxelgrid_size_live,
                                   double voxelgrid_size_merge,
                                   double min_distance,
                                   double max_distance,
                                   bool start_paused) :
    tf_buffer_(ros::Duration(std::numeric_limits<int>::max(), 0)),
    merged_cloud_(new pointcloud_t()),
    viewer_(viewer),
    voxelgrid_size_live_(voxelgrid_size_live),
    voxelgrid_size_merged_(voxelgrid_size_merge),
    min_distance_(min_distance),
    max_distance_(max_distance),
    pause_(start_paused)
  {
    merged_cloud_->header.stamp = 0;
    addCloudToViewer(merged_cloud_, viewer_);
  }


void CloudAccumulator::pointCloudCallback(const pointcloud_t::ConstPtr& pointcloud)
{
  if(pause_){ return; }
  ROS_INFO_THROTTLE(10, "Received %zu. point cloud", cloud_map_.size()+1);
  double timestamp = pointcloud->header.stamp * 1e-6;//pcl stamps are in milliseconds
  ROS_DEBUG("Storing cloud (%.3f)", timestamp);

  //Create the filtering object to create along the optical axis (Z)
  pointcloud_t::Ptr distance_filtered_cloud(new pointcloud_t());
  pcl::PassThrough<point_t> pass;
  pass.setInputCloud(pointcloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_distance_, max_distance_);
  pass.filter(*distance_filtered_cloud);
  cloud_map_[timestamp] = distance_filtered_cloud;//store for later corrections

  const Eigen::Affine3f transformation = lookupTransform(timestamp);
  if(transformation.matrix()(0,0) == transformation.matrix()(0,0)) //not nan
  {
    pointcloud_t::Ptr transformed_cloud(new pointcloud_t());
    pointcloud_t::Ptr voxel_filtered_cloud(new pointcloud_t());
    if(voxelgrid_size_live_ > 0.0)
    {
      //Voxelgridfilter to speedup display
      pcl::VoxelGrid<point_t> vgf;
      vgf.setInputCloud(distance_filtered_cloud);
      vgf.setLeafSize(voxelgrid_size_live_, voxelgrid_size_live_, voxelgrid_size_live_);
      vgf.filter(*voxel_filtered_cloud);
    }
    else
    {
      voxel_filtered_cloud = distance_filtered_cloud;//just copy the pointer
    }
    pcl::transformPointCloud(*voxel_filtered_cloud, *transformed_cloud, transformation);

    {
      boost::mutex::scoped_lock guard(merged_cloud_mutex_);
      *merged_cloud_ += *transformed_cloud;
      merged_cloud_->header.stamp = 0;
      updateCloudInViewer(merged_cloud_, viewer_);
    }
  }
  else
  {
    ROS_INFO_THROTTLE(5, "Cannot display point cloud, the (live) camera pose is not available. "
                      "Is Stereo INS or SLAM running?");
    ROS_DEBUG("No (live) camera pose available for cloud at %.3f.", timestamp);
  }
}


void CloudAccumulator::trajectoryCallback(const nav_msgs::PathConstPtr& path)
{
  if(pause_){ return; }
  ROS_INFO("Received Path with %zu poses", path->poses.size());
  boost::mutex::scoped_lock guard(tf_buffer_mutex_);
  tf_buffer_.clear();

  for(size_t i = 0; i < path->poses.size(); ++i)
  //for(const geometry_msgs::PoseStamped& pose : path->poses)
  {
    tf_buffer_.setTransform(myPoseStampedMsgToTF(path->poses[i]), "live_pose");
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


bool CloudAccumulator::mergeClouds(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp)
{
  ROS_INFO("Merging all stored clouds with latest poses.");
  pointcloud_t::Ptr merged(new pointcloud_t());
  int counter = 0;
  for(std::map<double, pointcloud_t::ConstPtr>::iterator it = cloud_map_.begin(); it != cloud_map_.end(); ++it)
  {
    const Eigen::Affine3f transformation = lookupTransform(it->first);
    if(transformation.matrix()(0,0) == transformation.matrix()(0,0)) //not nan
    {
      ++counter;
      pointcloud_t transformed_cloud;
      pcl::transformPointCloud(*(it->second), transformed_cloud, transformation);
      boost::mutex::scoped_lock guard(merged_cloud_mutex_);
      *merged += transformed_cloud;
    }
    else { ROS_DEBUG("Skipped cloud at %.3f in merge, cannot get its position", it->first); }
  }

  if(voxelgrid_size_merged_ > 0.0)
  {
    //Voxelgridfilter to minimize point redundancy
    pcl::VoxelGrid<point_t> vgf;
    vgf.setInputCloud(merged);
    vgf.setLeafSize(voxelgrid_size_merged_, voxelgrid_size_merged_, voxelgrid_size_merged_);
    viewer_->removeAllPointClouds();
    vgf.filter(*merged_cloud_);
  }
  else
  {
    viewer_->removeAllPointClouds();
    merged_cloud_ = merged;//just copy the pointer
  }

  ROS_INFO("(Re-)merged %d/%zu clouds.", counter, cloud_map_.size());
  merged_cloud_->header.stamp = 0;
  addCloudToViewer(merged_cloud_, viewer_);
  return true;
}


bool CloudAccumulator::togglePause(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp)
{
  pause_ = !pause_;
  return true;
}

}//namespace rc
