#!/usr/bin/env python3
"""
ROS 2 node that publishes KITTI Odometry LiDAR scans and ground-truth poses.

This module defines :class:`KittiPublisher`, a ROS 2 node that streams Velodyne
binary scans from a KITTI Odometry sequence and publishes:

  • ``sensor_msgs/PointCloud2`` for each LiDAR scan  
  • ``geometry_msgs/PoseStamped`` for the corresponding ground-truth vehicle pose  
  • A TF transform from a configurable base frame to the LiDAR frame

Dataset assumptions:
  * Directory layout follows the official KITTI Odometry structure. For a given
    ``dataset_sequence`` (e.g., ``"00"``), the expected paths are::

        <kitti_data_dir>/
          data_odometry_velodyne/dataset/sequences/<SEQ>/velodyne/*.bin
          data_odometry_poses/dataset/poses/<SEQ>.txt

  * Each ``.bin`` file encodes points as float32 quadruplets (x, y, z, intensity).
  * The poses file contains one row per frame with 12 float values corresponding
    to a 3×4 matrix [R|t] in row-major order.

Parameters (declare with ROS 2 parameters):
  kitti_data_dir (str):
      Root folder that contains both the Velodyne and poses subtrees.
      Default: ``/home/zeid/github/slam_frontend_ws/data/kitti``.
  publish_rate_hz (float):
      Timer frequency used to publish scans and poses in lock-step. Default: 10.0.
  pointcloud_topic (str):
      Topic for raw point clouds. Default: ``/kitti/pointcloud_raw``.
  ground_truth_pose_topic (str):
      Topic for ground-truth poses. Default: ``/kitti/ground_truth_pose``.
  base_frame_id (str):
      Parent frame used for TF and the PoseStamped header (e.g., ``map`` or ``odom``).
      Default: ``odom``.
  lidar_frame_id (str):
      Child frame and PointCloud2 header frame (e.g., ``velodyne``). Default: ``velodyne``.
  dataset_sequence (str):
      KITTI sequence identifier (two-digit string such as ``"00"``). Default: ``"00"``.

Published interfaces:
  * ``sensor_msgs/PointCloud2`` on ``pointcloud_topic`` with fields
    ``x``, ``y``, ``z``, and ``intensity`` (all ``FLOAT32``), ``point_step = 16``.
  * ``geometry_msgs/PoseStamped`` on ``ground_truth_pose_topic`` with
    ``header.frame_id = base_frame_id``.
  * TF transform broadcast from ``base_frame_id`` → ``lidar_frame_id``.

Example
-------
Run inside an environment where ROS 2 is sourced and your workspace is built:

.. code-block:: bash

    source /opt/ros/humble/setup.bash
    source ~/github/slam_frontend_ws/install/setup.bash
    ros2 run <your_pkg> kitti_publisher \\
      --ros-args \\
      -p kitti_data_dir:=/data/kitti \\
      -p dataset_sequence:=s00 \\
      -p base_frame_id:=map \\
      -p lidar_frame_id:=velodyne \\
      -p publish_rate_hz:=5.0

Notes
-----
* The node iterates once over all scans in the selected sequence. When the last
  scan is published, the timer is canceled and the node stops publishing.
* If required files or directories are missing, the node logs an error and
  shuts down cleanly.
"""

from __future__ import annotations  # Add this line

import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros


class KittiPublisher(Node):
    """
    ROS 2 node that publishes KITTI LiDAR scans and ground-truth poses in lock-step.

    The node lists all Velodyne ``.bin`` files for the selected KITTI sequence and
    loads the associated ground-truth poses. On each timer event it:

      1. Reads the next scan, converts it to ``sensor_msgs/PointCloud2``, and publishes it
         with ``header.frame_id = lidar_frame_id``.
      2. Publishes the corresponding ground-truth pose as ``geometry_msgs/PoseStamped``
         with ``header.frame_id = base_frame_id``.
      3. Broadcasts the same transform via TF from ``base_frame_id`` to ``lidar_frame_id``.

    Attributes:
      velodyne_path (str): Absolute path to ``.../sequences/<SEQ>/velodyne``.
      poses_path (str): Absolute path to ``.../poses/<SEQ>.txt``.
      scan_files (list[str]): Sorted list of Velodyne ``.bin`` files.
      ground_truth_poses (np.ndarray | None): Array of shape (N, 3, 4) with [R|t].
      base_frame (str): Parent frame used for TF and pose messages.
      lidar_frame (str): Child frame used for TF and PointCloud2 headers.
      dataset_sequence (str): KITTI sequence identifier (e.g., ``"00"``).
      pc_publisher (rclpy.publisher.Publisher): Publisher for point clouds.
      pose_publisher (rclpy.publisher.Publisher): Publisher for poses.
      tf_broadcaster (tf2_ros.TransformBroadcaster): TF broadcaster instance.
      scan_index (int): Current zero-based scan index.

    Design considerations:
      * The publishing timer frequency is configurable via ``publish_rate_hz``.
      * File system errors are surfaced through ROS logs, and the node exits gracefully.
      * SciPy is used to convert rotation matrices to quaternions for TF and poses.
    """

    def __init__(self) -> None:
        """
        Construct the node, declare parameters, resolve paths, validate inputs,
        allocate publishers, and start the publishing timer.

        Raises:
          SystemExit: The node calls ``rclpy.shutdown()`` and returns early if data
          cannot be loaded due to missing files or invalid paths.
        """
        super().__init__("kitti_publisher")

        # Declare parameters
        self.declare_parameter(
            "kitti_data_dir", "/home/zeid/github/slam_frontend_ws/data/kitti"
        )
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("pointcloud_topic", "/kitti/pointcloud_raw")
        self.declare_parameter("ground_truth_pose_topic", "/kitti/ground_truth_pose")
        self.declare_parameter("base_frame_id", "odom")
        self.declare_parameter("lidar_frame_id", "velodyne")
        self.declare_parameter("dataset_sequence", "s00")

        # Get parameters
        kitti_dir = (
            self.get_parameter("kitti_data_dir").get_parameter_value().string_value
        )
        publish_rate = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        pointcloud_topic = (
            self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        )
        pose_topic = (
            self.get_parameter("ground_truth_pose_topic")
            .get_parameter_value()
            .string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame_id").get_parameter_value().string_value
        )
        self.lidar_frame = (
            self.get_parameter("lidar_frame_id").get_parameter_value().string_value
        )
        # Get the "s00" or "s01" string
        seq_param_string = (
            self.get_parameter("dataset_sequence").get_parameter_value().string_value
        )

        # Strip the 's' prefix if it exists, otherwise use the string as-is
        if seq_param_string.startswith('s'):
            self.dataset_sequence = seq_param_string[1:] # e.g., "s01" -> "01"
        else:
            self.dataset_sequence = seq_param_string # e.g., "00"

        # Setup paths
        self.velodyne_path = os.path.join(
            kitti_dir,
            "data_odometry_velodyne",
            "dataset",
            "sequences",
            str(self.dataset_sequence),
            "velodyne",
        )
        self.poses_path = os.path.join(
            kitti_dir,
            "data_odometry_poses",
            "dataset",
            "poses",
            str(self.dataset_sequence) + ".txt",
        )
        self.get_logger().warn(self.poses_path)

        self.get_logger().info(f"Velodyne path: {self.velodyne_path}")
        self.get_logger().info(f"Poses path: {self.poses_path}")

        # Load data
        self.scan_files = sorted(os.listdir(self.velodyne_path))
        self.ground_truth_poses = self._load_poses()

        if not self.scan_files or self.ground_truth_poses is None:
            self.get_logger().error("Failed to load KITTI data. Check paths.")
            rclpy.shutdown()
            return

        # Initialize publishers and TF broadcaster
        self.pc_publisher = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize scan index and timer
        self.scan_index = 0
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info("KITTI Publisher node initialized.")

    def _load_poses(self) -> np.ndarray | None:
        """
        Load ground-truth poses from the KITTI poses text file.

        Returns:
          numpy.ndarray | None: Array of shape (N, 3, 4) containing per-frame
          [R|t] matrices if successful; ``None`` if the file cannot be read.

        File format:
          Each line has 12 float values, row-major, representing a 3×4 matrix.
        """
        try:
            poses = np.loadtxt(self.poses_path)
            return poses.reshape(-1, 3, 4)
        except FileNotFoundError:
            self.get_logger().error(f"Poses file not found at {self.poses_path}")
            return None

    def timer_callback(self) -> None:
        """
        Publish the next scan and its ground-truth pose on each timer tick.

        Behavior:
          * If all scans have been published, cancel the timer and log completion.
          * Otherwise, publish the next ``PointCloud2``, ``PoseStamped``, and TF.
        """
        if self.scan_index >= len(self.scan_files):
            self.get_logger().info("All KITTI scans have been published.")
            self.timer.cancel()
            return

        # Publish Point Cloud
        self._publish_point_cloud()

        # Publish Ground Truth Pose and Transform
        self._publish_ground_truth()

        self.scan_index += 1

    def _publish_point_cloud(self) -> None:
        """
        Read the current Velodyne ``.bin`` file and publish it as ``PointCloud2``.

        Message definition:
          * Fields: ``x``, ``y``, ``z``, ``intensity`` (all ``FLOAT32``)
          * ``point_step = 16`` bytes, ``row_step = point_step * width``
          * ``height = 1`` for an unorganized cloud

        Frame:
          The message header uses ``lidar_frame`` as ``frame_id``.
        """
        scan_path = os.path.join(self.velodyne_path, self.scan_files[self.scan_index])
        points = np.fromfile(scan_path, dtype=np.float32).reshape(-1, 4)

        header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id=self.lidar_frame
        )

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=16 * len(points),
            data=points.tobytes(),
        )
        self.pc_publisher.publish(pc2_msg)

    def _publish_ground_truth(self) -> None:
        """
        Publish the ground-truth pose and corresponding TF for the current index.

        Frames:
          Parent frame: ``base_frame`` (for example, ``map`` or ``odom``)
          Child frame : ``lidar_frame`` (for example, ``velodyne``)

        Implementation details:
          * The 3×4 [R|t] matrix is extended to 4×4 homogeneous form to extract
            translation and rotation.
          * SciPy converts the rotation matrix to a quaternion with ordering
            ``(x, y, z, w)`` suitable for ROS messages.

        Notes:
          If the poses array is shorter than the number of scans, this function
          returns without publishing for out-of-range indices.
        """
        if self.scan_index >= len(self.ground_truth_poses):
            return

        pose_matrix = np.vstack(
            [self.ground_truth_poses[self.scan_index], [0, 0, 0, 1]]
        )
        translation = pose_matrix[:3, 3]

        # SciPy conversion from rotation matrix to quaternion (x, y, z, w)
        from scipy.spatial.transform import Rotation

        quat = Rotation.from_matrix(pose_matrix[:3, :3]).as_quat()

        current_time = self.get_clock().now().to_msg()

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = Header(stamp=current_time, frame_id=self.base_frame)
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.pose_publisher.publish(pose_msg)

        # Broadcast TransformStamped (TF)
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """
    Initialize rclpy, create and spin :class:`KittiPublisher`, and shut down cleanly.

    Args:
      args (list[str] | None): Optional CLI arguments forwarded to rclpy.

    Behavior:
      * Interrupting with Ctrl-C stops spinning, destroys the node, and shuts down rclpy.
    """
    rclpy.init(args=args)
    kitti_publisher = KittiPublisher()
    try:
        rclpy.spin(kitti_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        kitti_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
