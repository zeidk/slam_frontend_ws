#!/usr/bin/env python3
"""LOAM-Style Feature Extraction for LiDAR Point Clouds.

This module implements curvature-based feature extraction as described in
the LOAM paper (Zhang & Singh, 2014) and taught in ENPM818Z L2C lecture.

The algorithm:
1. Organizes points into scan rings (laser channels)
2. Computes local curvature for each point
3. Selects edge features (high curvature) and planar features (low curvature)
4. Publishes separate point clouds for each feature type

Reference: ENPM818Z L2C, Section "Feature Extraction - Curvature-Based"
"""

from typing import Optional, List, Tuple, Dict
import struct

import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class LOAMFeatureExtractor(Node):
    """ROS 2 node for LOAM-style feature extraction.

    Extracts edge and planar features from LiDAR point clouds using
    curvature-based selection as described in the LOAM algorithm.

    Attributes:
        num_rings: Number of laser rings (16 for VLP-16, 64 for HDL-64)
        neighbor_size: Neighborhood size for curvature (typically 5)
        edge_threshold: Curvature threshold for edge features
        planar_threshold: Curvature threshold for planar features
        edge_percentage: Percentage of points to select as edges per ring
        planar_percentage: Percentage of points to select as planar per ring
    """

    def __init__(self) -> None:
        """Initialize the feature extractor node."""
        super().__init__("loam_feature_extractor")

        # Declare parameters based on L2C lecture
        self.declare_parameter("num_rings", 16)  # VLP-16 has 16 laser beams
        self.declare_parameter("neighbor_size", 5)  # m=5 => |S|=10 in lecture
        self.declare_parameter("edge_percentage", 2.0)  # Top 2%
        self.declare_parameter("planar_percentage", 2.0)  # Bottom 2%
        self.declare_parameter("min_range", 1.0)  # Min valid range (meters)
        self.declare_parameter("max_range", 100.0)  # Max valid range (meters)
        self.declare_parameter("input_topic", "preprocessing/downsampled_cloud")
        self.declare_parameter("edge_topic", "features/edge_cloud")
        self.declare_parameter("planar_topic", "features/planar_cloud")

        # Get parameters
        self.num_rings: int = self.get_parameter("num_rings").value
        self.neighbor_size: int = self.get_parameter("neighbor_size").value
        self.edge_percentage: float = self.get_parameter("edge_percentage").value
        self.planar_percentage: float = self.get_parameter("planar_percentage").value
        self.min_range: float = self.get_parameter("min_range").value
        self.max_range: float = self.get_parameter("max_range").value

        input_topic: str = self.get_parameter("input_topic").value
        edge_topic: str = self.get_parameter("edge_topic").value
        planar_topic: str = self.get_parameter("planar_topic").value

        # Setup QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, qos
        )

        # Create publishers for edge and planar features
        self.edge_publisher = self.create_publisher(PointCloud2, edge_topic, qos)

        self.planar_publisher = self.create_publisher(PointCloud2, planar_topic, qos)

        self.get_logger().info(
            f"LOAM Feature Extractor initialized:\n"
            f"  Number of rings: {self.num_rings}\n"
            f"  Neighbor size (m): {self.neighbor_size}\n"
            f"  Edge selection: top {self.edge_percentage}%\n"
            f"  Planar selection: bottom {self.planar_percentage}%\n"
            f"  Range filter: [{self.min_range}, {self.max_range}]m\n"
            f"  Subscribing to: {input_topic}\n"
            f"  Publishing edges to: {edge_topic}\n"
            f"  Publishing planar to: {planar_topic}"
        )

    def pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 to numpy array.

        Args:
            msg: Input PointCloud2 message

        Returns:
            Nx4 numpy array [x, y, z, intensity]
        """
        point_step = msg.point_step
        num_points = msg.width * msg.height

        points = np.zeros((num_points, 4), dtype=np.float32)

        for i in range(num_points):
            offset = i * point_step
            x = struct.unpack_from("f", msg.data, offset + 0)[0]
            y = struct.unpack_from("f", msg.data, offset + 4)[0]
            z = struct.unpack_from("f", msg.data, offset + 8)[0]
            intensity = struct.unpack_from("f", msg.data, offset + 12)[0]

            points[i] = [x, y, z, intensity]

        return points

    def organize_into_rings(self, points: np.ndarray) -> Dict[int, np.ndarray]:
        """Organize point cloud into laser rings.

        For multi-beam LiDARs (e.g., VLP-16 with 16 beams), points are
        organized into rings based on vertical angle. This preserves the
        ring structure needed for curvature computation.

        Algorithm from L2C lecture:
        - Compute vertical angle for each point
        - Assign to ring based on angle
        - Sort points within each ring by azimuth

        Args:
            points: Nx4 array [x, y, z, intensity]

        Returns:
            Dictionary mapping ring_id -> points in that ring
        """
        # Filter by range first
        ranges = np.linalg.norm(points[:, :3], axis=1)
        valid_mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        points = points[valid_mask]

        if points.shape[0] == 0:
            return {}

        # Compute vertical angles
        # For Velodyne: vertical angle = arctan(z / sqrt(x^2 + y^2))
        xy_dist = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
        vertical_angles = np.arctan2(points[:, 2], xy_dist)

        # VLP-16 has ±15° FOV with 2° increments
        # Map angles to ring indices
        min_angle = -15.0 * np.pi / 180.0
        max_angle = 15.0 * np.pi / 180.0
        angle_range = max_angle - min_angle

        ring_indices = np.floor(
            (vertical_angles - min_angle) / angle_range * self.num_rings
        ).astype(int)

        # Clamp to valid range
        ring_indices = np.clip(ring_indices, 0, self.num_rings - 1)

        # Compute azimuth angles for sorting within rings
        azimuth = np.arctan2(points[:, 1], points[:, 0])

        # Organize into rings
        rings = {}
        for ring_id in range(self.num_rings):
            mask = ring_indices == ring_id
            ring_points = points[mask]
            ring_azimuth = azimuth[mask]

            if ring_points.shape[0] > 0:
                # Sort by azimuth (scanning order)
                sort_idx = np.argsort(ring_azimuth)
                rings[ring_id] = ring_points[sort_idx]

        return rings

    def compute_curvature(self, points: np.ndarray, index: int) -> float:
        r"""Compute LOAM curvature for a single point.

        Formula (LOAM paper / L2C lecture):

        .. math::

           c_i \;=\; \frac{1}{\lvert \mathcal{S} \rvert \, \lVert \mathbf{p}_i \rVert}
                     \left\lVert \sum_{j \in \mathcal{S}} \bigl(\mathbf{p}_i - \mathbf{p}_j\bigr) \right\rVert

        where :math:`\mathcal{S}` is the set of neighboring points consisting of
        :math:`m` points on each side (so :math:`\lvert \mathcal{S} \rvert = 2m`).

        Args:
            points: ``(N, 4)`` array of points in the ring ``[x, y, z, intensity]``.
            index: Index of the point in ``points`` for which curvature is computed.

        Returns:
            Curvature value (smoothness/roughness score) as a ``float``.
        """
        n = points.shape[0]
        m = self.neighbor_size

        # Need sufficient neighbors on both sides
        if index < m or index >= n - m:
            return np.inf  # Mark as invalid

        # Get neighborhood: m points on each side
        # S = {p_{i-m}, ..., p_{i-1}, p_{i+1}, ..., p_{i+m}}
        neighbor_indices = list(range(index - m, index)) + list(
            range(index + 1, index + m + 1)
        )

        p_i = points[index, :3]  # [x, y, z]
        neighbors = points[neighbor_indices, :3]

        # Compute ||p_i||
        norm_p_i = np.linalg.norm(p_i)

        if norm_p_i < 1e-6:
            return np.inf

        # Compute sum_{j in S} (p_i - p_j)
        differences = p_i - neighbors
        sum_diff = np.sum(differences, axis=0)

        # Compute curvature
        num_neighbors = len(neighbor_indices)
        curvature = np.linalg.norm(sum_diff) / (num_neighbors * norm_p_i)

        return curvature

    def extract_features_from_ring(
        self, ring_points: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Extract edge and planar features from a single ring.

        Algorithm from L2C lecture:
        1. Compute curvature for all points in ring
        2. Sort points by curvature
        3. Select top X% as edge features (high curvature)
        4. Select bottom Y% as planar features (low curvature)

        Args:
            ring_points: Nx4 array of points in the ring

        Returns:
            Tuple of (edge_features, planar_features)
        """
        n = ring_points.shape[0]

        if n < 2 * self.neighbor_size:
            return np.array([]).reshape(0, 4), np.array([]).reshape(0, 4)

        # Compute curvature for all points
        curvatures = np.zeros(n)
        for i in range(n):
            curvatures[i] = self.compute_curvature(ring_points, i)

        # Filter out invalid points (inf curvature)
        valid_mask = np.isfinite(curvatures)
        valid_indices = np.where(valid_mask)[0]

        if len(valid_indices) == 0:
            return np.array([]).reshape(0, 4), np.array([]).reshape(0, 4)

        valid_curvatures = curvatures[valid_mask]
        valid_points = ring_points[valid_mask]

        # Sort by curvature
        sort_idx = np.argsort(valid_curvatures)

        # Select edge features (highest curvature)
        num_edges = max(1, int(len(valid_points) * self.edge_percentage / 100.0))
        edge_indices = sort_idx[-num_edges:]
        edge_features = valid_points[edge_indices]

        # Select planar features (lowest curvature)
        num_planar = max(1, int(len(valid_points) * self.planar_percentage / 100.0))
        planar_indices = sort_idx[:num_planar]
        planar_features = valid_points[planar_indices]

        return edge_features, planar_features

    def array_to_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:
        """Convert numpy array to PointCloud2 message.

        Args:
            points: Nx4 array [x, y, z, intensity]
            header: ROS message header

        Returns:
            PointCloud2 message
        """
        msg = PointCloud2()
        msg.header = header

        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.data = points.tobytes()

        return msg

    def cloud_callback(self, msg: PointCloud2) -> None:
        """Process incoming point cloud and extract features.

        Args:
            msg: Input PointCloud2 message
        """
        # Convert to numpy array
        points = self.pointcloud2_to_array(msg)

        if points.shape[0] == 0:
            self.get_logger().warn("Received empty point cloud")
            return

        # Organize into rings
        rings = self.organize_into_rings(points)

        if not rings:
            self.get_logger().warn("No valid rings after organization")
            return

        # Extract features from each ring
        all_edge_features = []
        all_planar_features = []

        for ring_id, ring_points in rings.items():
            edge_features, planar_features = self.extract_features_from_ring(
                ring_points
            )

            if edge_features.shape[0] > 0:
                all_edge_features.append(edge_features)

            if planar_features.shape[0] > 0:
                all_planar_features.append(planar_features)

        # Combine features from all rings
        if all_edge_features:
            edge_cloud = np.vstack(all_edge_features)
        else:
            edge_cloud = np.array([]).reshape(0, 4)

        if all_planar_features:
            planar_cloud = np.vstack(all_planar_features)
        else:
            planar_cloud = np.array([]).reshape(0, 4)

        # Publish feature clouds
        if edge_cloud.shape[0] > 0:
            edge_msg = self.array_to_pointcloud2(edge_cloud, msg.header)
            self.edge_publisher.publish(edge_msg)

        if planar_cloud.shape[0] > 0:
            planar_msg = self.array_to_pointcloud2(planar_cloud, msg.header)
            self.planar_publisher.publish(planar_msg)

        # Log statistics
        total_input = points.shape[0]
        num_edges = edge_cloud.shape[0]
        num_planar = planar_cloud.shape[0]
        reduction = 100.0 * (1.0 - (num_edges + num_planar) / total_input)

        self.get_logger().info(
            f"Feature extraction: {total_input} -> "
            f"{num_edges} edges + {num_planar} planar "
            f"({reduction:.1f}% reduction, {len(rings)} rings)",
            throttle_duration_sec=1.0,
        )


def main(args: Optional[List[str]] = None) -> None:
    """Main entry point for feature extractor node.

    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)

    try:
        node = LOAMFeatureExtractor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
