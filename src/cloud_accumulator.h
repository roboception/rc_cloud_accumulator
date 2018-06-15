#define PCL_NO_PRECOMPILE
#include "cloud_visualizer.h"
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

typedef pcl::PointXYZRGB point_t;
typedef pcl::PointCloud<point_t> pointcloud_t;

namespace rc
{

/**
 * Demonstration for how to create a registered point cloud map using the
 * rc_visard.
 */
class CloudAccumulator
{

  public:
  CloudAccumulator(double voxelgrid_size_live,
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
   * - Display
   */
  pointcloud_t::Ptr rebuildCloud();

  /**
   * - rebuild cloud (unless live_only) and update viewer
   * - Save as .pcd to disk
   */
  bool saveCloud(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp);

  /** Toggle whether to ignore incoming data */
  bool togglePause(std_srvs::Empty::Request &req, std_srvs::Empty::Request &resp);

  private:
  Eigen::Affine3f lookupTransform(double timestamp);

  //Members
  std::map<double, pointcloud_t::ConstPtr> cloud_map_;
  boost::mutex tf_buffer_mutex_;
  tf2_ros::Buffer tf_buffer_;      ///< Stores the history of poses and allows to interpolate
  boost::mutex display_cloud_mutex_;
  pointcloud_t::Ptr display_cloud_;///< Cloud that is displayed
  double voxelgrid_size_live_;     ///< Filter size for the display cloud
  double voxelgrid_size_final_;    ///< Filter size for the saved cloud
  double min_distance_;            ///< Discard points closer than this
  double max_distance_;            ///< Discard points farther than this
  std::string output_filename_;
  bool live_only_;                 ///< If set, do not store clouds. On save, save display cloud
  bool pause_;                     ///< Whether to ignore input
  CloudVisualizer viewer_;
  boost::thread viewer_thread_;
};

}//namespace rc
