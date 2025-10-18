Configuration
=============

The KITTI data loader is configured using a YAML parameter file that defines
runtime behavior for the publisher node.

Parameter File
--------------

.. yaml-params:: ../../config/params.yaml
   :node: kitti_publisher_node

Parameter Descriptions
----------------------

kitti_data_dir
~~~~~~~~~~~~~~
Absolute path to the root directory containing KITTI dataset files. This directory
should contain both ``data_odometry_velodyne`` and ``data_odometry_poses`` subdirectories.

**Important:** Update this path to match your workspace location.

dataset_sequence
~~~~~~~~~~~~~~~~
Two-digit string identifier for the KITTI sequence to publish (e.g., ``'00'``, ``'01'``, etc.).
The KITTI Odometry benchmark provides sequences 00-21.

publish_rate_hz
~~~~~~~~~~~~~~~
Publishing frequency in Hertz. Controls how fast scans and poses are published.
Lower values are useful for visualization, higher values for testing algorithms.

pointcloud_topic
~~~~~~~~~~~~~~~~
ROS 2 topic name for publishing ``sensor_msgs/PointCloud2`` messages containing
LiDAR point clouds.

ground_truth_pose_topic
~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 topic name for publishing ``geometry_msgs/PoseStamped`` messages containing
ground-truth vehicle poses.

base_frame_id
~~~~~~~~~~~~~
Parent frame ID used in TF transforms and pose messages. Typically ``'odom'`` or ``'map'``.

lidar_frame_id
~~~~~~~~~~~~~~
Child frame ID used in TF transforms and PointCloud2 messages. Represents the
LiDAR sensor frame, typically ``'velodyne'``.