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

/**
 * Demonstration for how to create a registered point cloud map using the
 * rc_visard.
 */
class CloudAccumulator
{

  public:
  CloudAccumulator(visualizer_ptr_t viewer,
                   double voxelgrid_size_live,
                   double voxelgrid_size_final,
                   double min_distance,
                   double max_distance,
                   std::string output_filename,
                   bool live_only,
                   bool start_paused);

  /**
   * For each cloud
   * - Filter by distance along the optical axis
   * - Store filtered cloud for later saving (unless live_only)
   * - Transform to world coordinates according to live pose
   * - Merge with previous clouds
   * - Apply "live" voxel grid filter on merged cloud
   * - Update viewer with the result
   */
  void pointCloudCallback(const pointcloud_t::ConstPtr& pointcloud);

  /** Replace all saved camera poses by those in \p path */
  void trajectoryCallback(const nav_msgs::PathConstPtr& path);

  /** Saves camera pose */
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& current_pose);

  /**
   * For all saved clouds
   * - Merge
   * - Voxel-grid filter
   */
  pointcloud_t::Ptr rebuildCloud();

  /**
   * - rebuild cloud (unless live_only) and update viewer
   * - Save as .pcd to disk
   */
  bool saveFinalCloud(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp);

  /** Toggle whether to ignore incoming data */
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
  double voxelgrid_size_final_;
  double min_distance_;
  double max_distance_;
  std::string output_filename_;
  bool live_only_;
  bool pause_;
};

}//namespace rc
