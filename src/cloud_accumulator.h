#define PCL_NO_PRECOMPILE
#include <tf2_ros/buffer.h>
#include <map>
#include <boost/thread/mutex.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB point_t;
typedef pcl::PointCloud<point_t> pointcloud_t;
typedef pcl::visualization::PCLVisualizer visualizer_t;
typedef boost::shared_ptr<visualizer_t> visualizer_ptr_t;

namespace rc
{

class CloudAccumulator
{

  public:
  CloudAccumulator(visualizer_ptr_t viewer,
                   double voxelgrid_size_live,
                   double voxelgrid_size_merged,
                   double min_distance,
                   double max_distance,
                   bool start_paused);

  void pointCloudCallback(const pointcloud_t::ConstPtr& pointcloud);

  void trajectoryCallback(const nav_msgs::PathConstPtr& path);

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& current_pose);

  bool mergeClouds(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp);

  bool togglePause(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp);

  private:
  Eigen::Affine3f lookupTransform(double timestamp);

  //Members
  std::map<double, pointcloud_t::ConstPtr> cloud_map_;
  tf2_ros::Buffer tf_buffer_;
  boost::mutex tf_buffer_mutex_;
  pointcloud_t::Ptr merged_cloud_;
  boost::mutex merged_cloud_mutex_;
  visualizer_ptr_t viewer_;
  double voxelgrid_size_live_;
  double voxelgrid_size_merged_;
  double min_distance_;
  double max_distance_;
  bool pause_;
};

}//namespace rc
