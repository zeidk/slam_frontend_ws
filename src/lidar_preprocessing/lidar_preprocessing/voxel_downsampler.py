#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
voxel_downsampler.py
====================

A ROS 2 node that subscribes to a raw LiDAR `sensor_msgs/PointCloud2`, applies
voxel-grid downsampling with NumPy, and publishes a sanitized, consistent
`PointCloud2` containing fields [x, y, z, intensity] as float32.

Design goals
------------
1) Robustness to arbitrary PointCloud2 layouts.
   - Uses `sensor_msgs_py.point_cloud2.read_points`, which respects field
     offsets, padding, and extra fields (e.g., ring, time).

2) Safety for visualization tools.
   - Drops any rows containing NaN or Inf prior to processing.
   - Skips publishing if the result is empty.

3) Simple deployment.
   - No pandas dependency; only NumPy is required.

4) Friendly defaults for debugging.
   - Publishes with RELIABLE reliability by default so that
     `ros2 topic echo` works without QoS flags.
   - Subscriber uses the standard sensor-data QoS.

Parameters
----------
- input_topic : str (default: "kitti/raw_cloud")
    Topic to subscribe for incoming raw point clouds.

- output_topic : str (default: "/preprocessing/downsampled_cloud")
    Topic to publish the downsampled cloud.

- voxel_size : float (default: 0.2)
    Edge length in meters for cubic voxels. Must be positive.

- min_points_per_voxel : int (default: 1)
    Minimum number of points in a voxel required to keep its centroid.

- filter_ground : bool (default: False)
    If True, discard points with z <= ground_threshold prior to downsampling.

- ground_threshold : float (default: -1.2)
    Z height threshold used when filter_ground is True.

- output_qos_reliability : str (default: "reliable")
    Reliability for the publisher. One of {"reliable", "best_effort"}.
    "reliable" is recommended for easy introspection with `ros2 topic echo`.

Usage
-----
Launch as usual (parameters may be set in the launch file):

    ros2 launch lidar_preprocessing preprocessing.launch.py

Sanity checks
-------------
Ensure input arrives:

    ros2 topic echo -n 1 kitti/raw_cloud --qos-reliability best_effort

Inspect output header first:

    ros2 topic echo -n 1 /preprocessing/downsampled_cloud.header

Inspect QoS and connections:

    ros2 topic info /preprocessing/downsampled_cloud --verbose
"""

from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class VoxelDownsampler(Node):
    """
    ROS 2 node that performs voxel-grid downsampling on incoming PointCloud2 messages.

    The node:
      1) Subscribes to `input_topic` with a sensor-data QoS profile.
      2) Converts the incoming cloud to a dense NumPy array of shape (N, 4)
         with columns [x, y, z, intensity]. If intensity is missing, it will
         be synthesized as ones.
      3) Optionally filters ground by z height.
      4) Downsamples via voxel-grid centroids with averaging of intensity.
      5) Publishes a PointCloud2 with fields [x, y, z, intensity] as float32.

    The output message inherits `frame_id` and `stamp` from the input by default.
    """

    def __init__(self) -> None:
        super().__init__("voxel_downsampler")

        # ----------------------------
        # Declare parameters with defaults
        # ----------------------------
        self.declare_parameter("input_topic", "kitti/raw_cloud")
        self.declare_parameter("output_topic", "/preprocessing/downsampled_cloud")
        self.declare_parameter("voxel_size", 0.2)
        self.declare_parameter("min_points_per_voxel", 1)
        self.declare_parameter("filter_ground", False)
        self.declare_parameter("ground_threshold", -1.2)
        self.declare_parameter("output_qos_reliability", "reliable")

        # ----------------------------
        # Read parameters
        # ----------------------------
        self.input_topic: str = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic: str = self.get_parameter("output_topic").get_parameter_value().string_value
        self.voxel_size: float = float(self.get_parameter("voxel_size").get_parameter_value().double_value)
        self.min_points_per_voxel: int = int(self.get_parameter("min_points_per_voxel").get_parameter_value().integer_value)
        self.filter_ground: bool = bool(self.get_parameter("filter_ground").get_parameter_value().bool_value)
        self.ground_threshold: float = float(self.get_parameter("ground_threshold").get_parameter_value().double_value)
        qos_str: str = self.get_parameter("output_qos_reliability").get_parameter_value().string_value.lower().strip()

        # Parameter validation with guard rails.
        if self.voxel_size <= 0.0:
            self.get_logger().warn("Parameter 'voxel_size' must be > 0. Setting to 0.2 m.")
            self.voxel_size = 0.2
        if self.min_points_per_voxel < 1:
            self.get_logger().warn("Parameter 'min_points_per_voxel' must be >= 1. Setting to 1.")
            self.min_points_per_voxel = 1
        if qos_str not in {"reliable", "best_effort"}:
            self.get_logger().warn("Parameter 'output_qos_reliability' must be 'reliable' or 'best_effort'. Defaulting to 'reliable'.")
            qos_str = "reliable"

        # ----------------------------
        # Build QoS profile for publisher
        # ----------------------------
        # Use RELIABLE by default so `ros2 topic echo` works without flags.
        qos_out = QoSProfile(depth=10)
        qos_out.durability = DurabilityPolicy.VOLATILE
        if qos_str == "reliable":
            qos_out.reliability = ReliabilityPolicy.RELIABLE
        else:
            qos_out.reliability = ReliabilityPolicy.BEST_EFFORT

        # ----------------------------
        # Create subscriber and publisher
        # ----------------------------
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._on_cloud, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_out)

        # Basic counters for light diagnostics.
        self._num_in: int = 0
        self._num_out: int = 0

        self.get_logger().info(
            "VoxelDownsampler configuration:\n"
            f"  input_topic: {self.input_topic}\n"
            f"  output_topic: {self.output_topic}\n"
            f"  voxel_size: {self.voxel_size:.3f} m\n"
            f"  min_points_per_voxel: {self.min_points_per_voxel}\n"
            f"  filter_ground: {self.filter_ground}\n"
            f"  ground_threshold: {self.ground_threshold:.3f} m\n"
            f"  output_qos_reliability: {qos_str.upper()}"
        )

    # =========================================================================
    # Callback
    # =========================================================================

    def _on_cloud(self, msg: PointCloud2) -> None:
        """
        Handle an incoming PointCloud2 message.

        Steps:
          1) Convert to NumPy array [x, y, z, intensity] float32.
          2) Sanitize (drop non-finite rows).
          3) Optional ground filtering by z height.
          4) Voxel downsample.
          5) Publish result with same frame_id and timestamp.

        The method is exception-safe; it logs and skips a bad message rather than crashing.
        """
        self._num_in += 1
        try:
            # Convert arbitrary PointCloud2 layout to a dense Nx4 array.
            pts = self._pc2_to_array(msg)
            if pts.size == 0:
                # Nothing to process; skip publishing.
                return

            # Optional ground filter.
            if self.filter_ground:
                pts = pts[pts[:, 2] > self.ground_threshold]
                if pts.size == 0:
                    return

            # Perform voxel-grid downsampling.
            down = self._voxel_downsample(pts, self.voxel_size, self.min_points_per_voxel)
            if down.size == 0:
                return

            # Build outgoing cloud. Reuse input frame and stamp for temporal coherence.
            out_msg = self._array_to_pc2(
                down, frame_id=msg.header.frame_id, stamp=msg.header.stamp
            )
            self.pub.publish(out_msg)
            self._num_out += 1

            # Informative log every 30 messages to avoid spam.
            if self._num_in % 30 == 0:
                self.get_logger().info(
                    f"[{self._num_in} in / {self._num_out} out] "
                    f"in_pts={pts.shape[0]} out_pts={down.shape[0]} frame='{msg.header.frame_id}'"
                )

        except Exception as ex:
            # Log exception and continue running (do not crash the node).
            self.get_logger().error(f"Exception while processing cloud #{self._num_in}: {ex!r}")

    # =========================================================================
    # PointCloud2 <-> NumPy
    # =========================================================================

    def _pc2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """
        Convert an arbitrary `sensor_msgs/PointCloud2` to an array of shape (N, 4) float32.

        Output columns are [x, y, z, intensity]. If `intensity` is not present in the
        incoming message, it will be synthesized as ones to maintain a consistent layout.

        All rows containing NaN or Inf are removed. If the result is empty, an empty
        (0, 4) float32 array is returned.

        Parameters
        ----------
        msg : PointCloud2
            Input point cloud.

        Returns
        -------
        np.ndarray
            Array of shape (N, 4) with dtype float32.
        """
        # Derive available field names from message meta-data.
        field_names = [f.name for f in msg.fields]
        has_intensity = "intensity" in field_names

        if has_intensity:
            # Respect actual field offsets and padding. Skip NaNs provided by the generator.
            gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
            arr = np.array(list(gen), dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(-1, 4)
        else:
            # Fall back to xyz and synthesize intensity = 1.0
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            xyz = np.array(list(gen), dtype=np.float32)
            if xyz.size == 0:
                return xyz.reshape(0, 4)
            if xyz.ndim == 1:
                xyz = xyz.reshape(-1, 3)
            intensity = np.ones((xyz.shape[0], 1), dtype=np.float32)
            arr = np.concatenate([xyz, intensity], axis=1)

        # Remove any rows that still contain non-finite values (defensive).
        finite = np.isfinite(arr).all(axis=1)
        arr = arr[finite]
        if arr.size == 0:
            return arr.reshape(0, 4)

        return arr

    def _array_to_pc2(
        self,
        pts: np.ndarray,
        frame_id: Optional[str] = None,
        stamp: Optional[Time] = None,
    ) -> PointCloud2:
        """
        Convert an array of shape (N, 4) float32 to a `sensor_msgs/PointCloud2`.

        The output message contains fields:
          - x (float32) at offset 0
          - y (float32) at offset 4
          - z (float32) at offset 8
          - intensity (float32) at offset 12

        Parameters
        ----------
        pts : np.ndarray
            Array of shape (N, 4) and dtype float32 containing [x, y, z, intensity].
        frame_id : Optional[str]
            Output frame id. If None, "map" is used.
        stamp : Optional[Time]
            Header stamp to use. If None, the current node time is used.

        Returns
        -------
        PointCloud2
            Constructed ROS message ready for publishing.
        """
        # Validate basic structure of the input array.
        assert pts.ndim == 2 and pts.shape[1] == 4, "Expected pts to be (N, 4) as [x, y, z, intensity]"
        assert pts.dtype == np.float32, "Expected pts dtype float32"

        # Explicit field layout for stability across consumers.
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Compose header.
        header = Header()
        header.frame_id = frame_id if frame_id else "map"
        header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()

        # pc2.create_cloud expects an iterable of tuples or lists.
        # Converting to a list of tuples once is simple and readable.
        points_iter = [tuple(row) for row in pts.tolist()]

        return pc2.create_cloud(header, fields, points_iter)

    # =========================================================================
    # Voxel downsampling
    # =========================================================================

    @staticmethod
    def _voxel_downsample(
        pts: np.ndarray,
        voxel_size: float,
        min_points_per_voxel: int = 1,
    ) -> np.ndarray:
        """
        Downsample via voxel-grid centroid aggregation.

        Voxel index is computed as:
            idx = floor([x, y, z] / voxel_size)

        For each unique voxel, compute the centroid of [x, y, z, intensity].
        Optionally reject voxels with fewer than `min_points_per_voxel` points.

        Parameters
        ----------
        pts : np.ndarray
            Array of shape (N, 4) float32: [x, y, z, intensity].
        voxel_size : float
            Positive voxel edge length in meters.
        min_points_per_voxel : int
            Minimum number of points to keep a voxel.

        Returns
        -------
        np.ndarray
            Downsampled array of shape (M, 4) float32.
        """
        if pts.size == 0:
            return pts.reshape(0, 4)
        if voxel_size <= 0.0:
            # Invalid setting; return input unchanged rather than failing.
            return pts

        # Compute integer voxel indices for each point (N, 3) int32.
        idx = np.floor(pts[:, :3] / voxel_size).astype(np.int32)

        # Unique voxel keys and inverse map: for each point -> its voxel id in [0..K-1].
        keys, inv = np.unique(idx, axis=0, return_inverse=True)

        if min_points_per_voxel > 1:
            # Count points per voxel, keep only voxels with sufficient support.
            counts_all = np.bincount(inv)
            keep_voxel = counts_all >= min_points_per_voxel
            if not np.any(keep_voxel):
                return np.empty((0, 4), dtype=np.float32)

            # Remap voxel ids so that kept voxels are compacted to [0..M-1].
            mapping = -np.ones(keep_voxel.size, dtype=np.int64)
            mapping[np.nonzero(keep_voxel)[0]] = np.arange(int(np.count_nonzero(keep_voxel)))
            inv2 = mapping[inv]
            keep_points = inv2 != -1
            if not np.any(keep_points):
                return np.empty((0, 4), dtype=np.float32)

            pts = pts[keep_points]
            inv = inv2[keep_points]
            n_vox = int(inv.max()) + 1
        else:
            n_vox = keys.shape[0]

        # Sum coordinates per voxel for centroid computation.
        sums = np.zeros((n_vox, 4), dtype=np.float64)
        for c in range(4):
            sums[:, c] = np.bincount(inv, weights=pts[:, c], minlength=n_vox)

        counts = np.bincount(inv, minlength=n_vox).astype(np.float64)
        nonzero = counts > 0.0
        if not np.any(nonzero):
            return np.empty((0, 4), dtype=np.float32)

        centroids = np.zeros_like(sums, dtype=np.float64)
        centroids[nonzero] = (sums[nonzero].T / counts[nonzero]).T

        return centroids.astype(np.float32)


# =========================================================================
# Entry point
# =========================================================================

def main(args=None) -> None:
    """
    Initialize rclpy, construct the node, and spin until shutdown.
    """
    rclpy.init(args=args)
    node = VoxelDownsampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow a clean exit when interrupted from the console.
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
