rc_cloud_accumulator
====================

This project demonstrates how to create a registered point cloud map
using the roboception rc_visard with the ROS driver.

What it does
------------
The rc_cloud_accumulator ROS node subscribes to the following topics of the *rc_visard_driver*

 - /stereo/points2
 - /pose
 - /trajectory

The received information is stored, such that a registered point cloud can be
computed and saved to disk. For performance reasons, the point clouds are preprocessed.
See *Filters* below for more details.

The node displays point clouds received on the first topic using the live pose (/pose)
to position them in the global coordinate frame.

The trajectory topic can be used to feed an optimized trajectory from the SLAM module
into the *rc_cloud_accumulator*. The easiest way is to start the *rc_visard_driver*
with the parameter *autopublish_trajectory* set to `True` and call the service 
*/rc_visard_driver/get_trajectory*. The *rc_visard_driver* will then send the
trajectory on */trajectory*.

Filters
-------

For performance reasons the point clouds are filtered in several stages.
The filter parameters are configurable via ROS parameters. See
*ROS Parameters* for a detailed description of the parameters.

When a point cloud is received, the points will first be filtered by
a minimum and maximum distance along the optical axis.

A copy of the resulting cloud will be stored in memory for later use.

The point cloud will then be transformed to the global coordinate
frame and merged into the currently displayed *live* point cloud. To keep the
visualization snappy, the *live* point cloud will be filtered with a voxel
grid using the parameter *voxel_grid_size_live*.

When the save_cloud service (see below) is used, the stored point clouds will
be merged (considering pose updates received in the meantime). The result will
also be be filtered with a voxel grid using the parameter
*voxel_grid_size_final*.


ROS Services
------------

The *rc_cloud_accumulator* provides the following services

 - save_cloud: Register stored clouds and save them to disk
 - toggle_pause: Toggle processing of received data

ROS Parameters
--------------

  - voxel_grid_size_live (default = 0.05m): Downsampling grid size of the point cloud in the live display
  - voxel_grid_size_final (default = 0.01m): Downsampling grid size of the point cloud when saving to disk
  - minimum_distance (default = 0.1m): Omit points closer to the rc_visard
  - maximum_distance (default = 5.0m): Omit points closer farther from the rc_visard
  - output_filename (default = "cloud.pcd")
  - start_paused (default = false)
  - live_only (default = false): Set to true to save memory and processing if no trajectories with corrections will be available.
    The original point clouds will not be stored and the point cloud saved to
    disk will be the one that is displayed. Only the voxel grid filter for the
    *live* pose will be applied.
