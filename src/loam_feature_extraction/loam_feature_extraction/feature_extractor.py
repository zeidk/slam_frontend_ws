#!/usr/bin/env python3


from typing import Optional, List, Tuple, Dict
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class LOAMFeatureExtractor(Node):
    """ROS 2 node for LOAM-style feature extraction."""

    def __init__(self) -> None:
        """
        Initialize the LOAM feature extraction node.

        This constructor declares/reads parameters, configures QoS, and sets up
        the subscriber (preprocessed cloud) and two publishers (edge/planar).
        A configuration summary is logged for traceability.

        Declared ROS Parameters:
            - num_rings (int): Number of beam rings.
            - neighbor_size (int): Neighborhood half-size m.
            - edge_percentage (float): % of highest-curvature points per ring.
            - planar_percentage (float): % of lowest-curvature points per ring.
            - min_range (float): Range filter minimum (m).
            - max_range (float): Range filter maximum (m).
            - min_vertical_deg (float): Lower bound of vertical FOV (deg).
            - max_vertical_deg (float): Upper bound of vertical FOV (deg).
            - input_topic (str): Preprocessed cloud input topic.
            - edge_topic (str): Edge cloud output topic.
            - planar_topic (str): Planar cloud output topic.

        Subscriptions:
            - input_topic : PointCloud2

        Publications:
            - edge_topic   : PointCloud2 (width may be 0)
            - planar_topic : PointCloud2 (width may be 0)
        """
        super().__init__("loam_feature_extractor")

        # Parameters and defaults
        self.declare_parameter("num_rings", 16)
        self.declare_parameter("neighbor_size", 5)
        self.declare_parameter("edge_percentage", 2.0)
        self.declare_parameter("planar_percentage", 2.0)
        self.declare_parameter("min_range", 1.0)
        self.declare_parameter("max_range", 100.0)
        self.declare_parameter("min_vertical_deg", -15.0)
        self.declare_parameter("max_vertical_deg", 15.0)

        self.declare_parameter("input_topic", "preprocessing/downsampled_cloud")
        self.declare_parameter("edge_topic", "features/edge_cloud")
        self.declare_parameter("planar_topic", "features/planar_cloud")

        # Fetch parameters
        self.num_rings: int = int(self.get_parameter("num_rings").value)
        self.neighbor_size: int = int(self.get_parameter("neighbor_size").value)
        self.edge_percentage: float = float(self.get_parameter("edge_percentage").value)
        self.planar_percentage: float = float(
            self.get_parameter("planar_percentage").value
        )
        self.min_range: float = float(self.get_parameter("min_range").value)
        self.max_range: float = float(self.get_parameter("max_range").value)
        self.min_vertical_deg: float = float(
            self.get_parameter("min_vertical_deg").value
        )
        self.max_vertical_deg: float = float(
            self.get_parameter("max_vertical_deg").value
        )

        input_topic: str = str(self.get_parameter("input_topic").value)
        edge_topic: str = str(self.get_parameter("edge_topic").value)
        planar_topic: str = str(self.get_parameter("planar_topic").value)

        # QoS: reliable, volatile, depth 10
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # I/O topics
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, qos
        )
        self.edge_publisher = self.create_publisher(PointCloud2, edge_topic, qos)
        self.planar_publisher = self.create_publisher(PointCloud2, planar_topic, qos)

        self.get_logger().info(
            "LOAM Feature Extractor initialized:\n"
            f"  num_rings={self.num_rings}\n"
            f"  neighbor_size(m)={self.neighbor_size}\n"
            f"  edge_percentage={self.edge_percentage}%\n"
            f"  planar_percentage={self.planar_percentage}%\n"
            f"  range=[{self.min_range}, {self.max_range}] m\n"
            f"  vertical FOV=[{self.min_vertical_deg}, {self.max_vertical_deg}] deg\n"
            f"  input_topic={input_topic}\n"
            f"  edge_topic={edge_topic}\n"
            f"  planar_topic={planar_topic}"
        )

    # ---------------------------------------------------------------------
    # PointCloud2 <-> NumPy Conversion
    # ---------------------------------------------------------------------
    def pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """
        Convert a ROS 2 PointCloud2 message to an (N, 4) float32 NumPy array.

        Assumes the point fields are laid out as 16 bytes per point:
        `[x(float32), y(float32), z(float32), intensity(float32)]`.

        Args:
            msg (sensor_msgs.msg.PointCloud2):
                Input point cloud (possibly organized or unorganized).

        Returns:
            np.ndarray:
                A contiguous array of shape (N, 4) with dtype float32, or an
                empty (0, 4) array if msg.width * msg.height == 0.

        Notes:
            - This function reads directly from `msg.data` using struct unpack.
            - `N = msg.width * msg.height`. For typical LiDAR clouds, height is 1.
        """
        n_pts = int(msg.width) * int(msg.height)
        if n_pts == 0:
            return np.empty((0, 4), dtype=np.float32)

        pts = np.empty((n_pts, 4), dtype=np.float32)
        step = int(msg.point_step)  # bytes per point

        # Offsets: 0:x, 4:y, 8:z, 12:intensity
        for i in range(n_pts):
            base = i * step
            x = struct.unpack_from("f", msg.data, base + 0)[0]
            y = struct.unpack_from("f", msg.data, base + 4)[0]
            z = struct.unpack_from("f", msg.data, base + 8)[0]
            intensity = struct.unpack_from("f", msg.data, base + 12)[0]
            pts[i] = (x, y, z, intensity)

        return pts

    def array_to_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:
        """
        Convert an (N, 4) float32 NumPy array to a ROS 2 PointCloud2 message.

        The output message is unorganized (height=1) and uses a 16-byte point
        layout `[x, y, z, intensity]`.

        Args:
            points (np.ndarray):
                Array of shape (N, 4) containing `[x, y, z, intensity]`. If N=0,
                an empty message is produced (width=0).
            header (std_msgs.msg.Header):
                Header copied from the input cloud (timestamp, frame_id).

        Returns:
            sensor_msgs.msg.PointCloud2:
                Serialized point cloud message, suitable for publishing.

        Notes:
            - `is_dense` is set to True; invalid points should be filtered earlier.
            - `row_step = point_step * N`.
        """
        msg = PointCloud2()
        msg.header = header
        msg.height = 1

        count = int(points.shape[0]) if points.ndim == 2 else 0
        msg.width = count

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
        msg.row_step = msg.point_step * count
        msg.data = (
            b"" if count == 0 else points.astype(np.float32, copy=False).tobytes()
        )
        return msg

    # ---------------------------------------------------------------------
    # Ring Organization and Curvature
    # ---------------------------------------------------------------------
    def organize_into_rings(self, points: np.ndarray) -> Dict[int, np.ndarray]:
        """
        Group points into rings using vertical angle and sort each ring by azimuth.

        Steps:
            1) Range-filter points to [min_range, max_range].
            2) Compute vertical angle `v = atan2(z, hypot(x, y))`.
            3) Map to ring index with configured vertical FOV:
               ring = floor((v - v_min) / (v_max - v_min) * num_rings), clamped.
            4) Within each ring, sort points by azimuth `atan2(y, x)`.

        Args:
            points (np.ndarray):
                Array of shape (N, 4) `[x, y, z, intensity]`.

        Returns:
            Dict[int, np.ndarray]:
                Mapping `ring_id -> (K, 4)` array for that ring (K may vary).
                Rings with no points are omitted.

        Notes:
            - Vertical FOV is controlled by `min_vertical_deg` and `max_vertical_deg`.
            - If all points are filtered, returns `{}`.
        """
        if points.size == 0:
            return {}

        # Range filter
        ranges = np.linalg.norm(points[:, :3], axis=1)
        mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        points = points[mask]
        if points.size == 0:
            return {}

        # Vertical angle and azimuth
        xy = np.hypot(points[:, 0], points[:, 1])
        v_ang = np.arctan2(points[:, 2], xy)
        az = np.arctan2(points[:, 1], points[:, 0])

        # Ring assignment via configured vertical FOV
        v_min = np.deg2rad(self.min_vertical_deg)
        v_max = np.deg2rad(self.max_vertical_deg)
        v_span = max(1e-6, v_max - v_min)

        ring_idx = np.floor((v_ang - v_min) / v_span * self.num_rings).astype(int)
        ring_idx = np.clip(ring_idx, 0, self.num_rings - 1)

        rings: Dict[int, np.ndarray] = {}
        for r in range(self.num_rings):
            sel = ring_idx == r
            if not np.any(sel):
                continue
            rp = points[sel]
            rp_az = az[sel]
            order = np.argsort(rp_az)
            rings[r] = rp[order]

        return rings

    def compute_curvature(self, ring_points: np.ndarray, idx: int, m: int) -> float:
        r"""
        Compute LOAM-style curvature for a point using a 2m neighborhood.

        Mathematical definition:

        .. math::

            c_i = \frac{\left\| \sum_{j \in S} (\mathbf{p}_i - \mathbf{p}_j) \right\|}{|S| \cdot \|\mathbf{p}_i\|}

        where:

            - :math:`\mathbf{p}_i` is the 3D position of the center point in the ring
            - :math:`S = \{i-m, \ldots, i-1, i+1, \ldots, i+m\}` (m neighbors on each side)
            - :math:`|S| = 2m`

        Args:
            ring_points (np.ndarray):
                Array of shape (K, 4) for a single ring (K points).
            idx (int):
                Index of the point within `ring_points` to evaluate.
            m (int):
                Neighborhood half-width (effective m; may be smaller on short rings).

        Returns:
            float:
                Curvature value. Returns `np.inf` if the point cannot be evaluated
                (e.g., near ring borders or zero-length vector).
        """
        n = ring_points.shape[0]
        if idx < m or idx >= n - m:
            return np.inf

        p_i = ring_points[idx, :3]
        norm_p_i = np.linalg.norm(p_i)
        if norm_p_i < 1e-6:
            return np.inf

        neighbors = np.vstack(
            (ring_points[idx - m : idx, :3], ring_points[idx + 1 : idx + m + 1, :3])
        )
        sum_diff = (p_i - neighbors).sum(axis=0)
        return float(np.linalg.norm(sum_diff) / (2 * m * norm_p_i))

    def extract_features_from_ring(
        self, ring_points: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Select edge and planar features from a single ring based on curvature.

        Pipeline (per ring):
            1) Compute curvature for all points using an effective neighborhood
               `m_eff = min(self.neighbor_size, (n-1)//2)` so short rings still work.
            2) Filter invalid curvatures (np.inf).
            3) Sort by curvature (ascending: low→high).
            4) Pick bottom `planar_percentage%` as planar; top `edge_percentage%` as edges.
               Enforce non-overlapping selections and at least 1 of each if any valid points.

        Args:
            ring_points (np.ndarray):
                (K, 4) array of `[x, y, z, intensity]` points for one ring.

        Returns:
            Tuple[np.ndarray, np.ndarray]:
                - edge_features  : (E, 4) array (may be (0, 4) if none)
                - planar_features: (P, 4) array (may be (0, 4) if none)

        Notes:
            - If `K < 3`, returns empty sets (not enough neighbors).
            - Counts are computed *after* filtering invalid curvatures.
            - Edge and planar selections are guaranteed non-overlapping.
        """
        n = ring_points.shape[0]
        if n < 3:
            return np.empty((0, 4), np.float32), np.empty((0, 4), np.float32)

        # Effective neighborhood that fits the ring length
        m_eff = min(self.neighbor_size, max(1, (n - 1) // 2))

        curv = np.empty(n, dtype=np.float32)
        for i in range(n):
            curv[i] = self.compute_curvature(ring_points, i, m_eff)

        valid_mask = np.isfinite(curv)
        if not np.any(valid_mask):
            return np.empty((0, 4), np.float32), np.empty((0, 4), np.float32)

        pts = ring_points[valid_mask]
        c = curv[valid_mask]

        order = np.argsort(c)  # ascending: low (planar) -> high (edge)
        pts_sorted = pts[order]

        n_valid = pts_sorted.shape[0]

        # Calculate desired counts based on percentages
        n_edge_desired = max(1, int(n_valid * (self.edge_percentage / 100.0)))
        n_plan_desired = max(1, int(n_valid * (self.planar_percentage / 100.0)))

        # Ensure non-overlapping selections: total cannot exceed available points
        total_desired = n_edge_desired + n_plan_desired
        if total_desired > n_valid:
            # Scale down but maintain the ratio between edge and planar
            scale_factor = n_valid / total_desired
            n_edge = max(1, int(n_edge_desired * scale_factor))
            n_plan = max(1, int(n_plan_desired * scale_factor))
            # Adjust if rounding caused the total to still exceed available points
            if n_edge + n_plan > n_valid:
                n_plan = n_valid - n_edge
        else:
            n_edge = n_edge_desired
            n_plan = n_plan_desired

        planar_features = pts_sorted[:n_plan]
        edge_features = pts_sorted[-n_edge:]

        return edge_features, planar_features

    # ---------------------------------------------------------------------
    # Main callback
    # ---------------------------------------------------------------------
    def cloud_callback(self, msg: PointCloud2) -> None:
        """
        Handle an incoming preprocessed cloud, extract features, and publish results.

        Steps:
            1) Convert PointCloud2 → NumPy array.
            2) Organize into rings (range filter + vertical FOV mapping + azimuth sort).
            3) Extract edge/planar features per ring.
            4) Concatenate features across rings.
            5) Publish both clouds (even if empty: width == 0).
            6) Log a one-per-second summary for quick monitoring.

        Args:
            msg (sensor_msgs.msg.PointCloud2):
                The incoming preprocessed/downsampled LiDAR cloud.

        Publishing Semantics:
            - Edge and planar messages are **always** published. If no points are
              selected for a type, the published message has `width = 0`. This is
              intentional to keep downstream plots (e.g., `.width`) continuous.
        """
        pts = self.pointcloud2_to_array(msg)
        if pts.size == 0:
            empty = self.array_to_pointcloud2(np.empty((0, 4), np.float32), msg.header)
            self.edge_publisher.publish(empty)
            self.planar_publisher.publish(empty)
            self.get_logger().warn(
                "Received empty cloud; published empty feature clouds."
            )
            return

        rings = self.organize_into_rings(pts)
        if not rings:
            empty = self.array_to_pointcloud2(np.empty((0, 4), np.float32), msg.header)
            self.edge_publisher.publish(empty)
            self.planar_publisher.publish(empty)
            self.get_logger().warn(
                "No valid rings after organization; published empty feature clouds."
            )
            return

        all_edges: List[np.ndarray] = []
        all_planar: List[np.ndarray] = []

        for _, ring_pts in rings.items():
            e, p = self.extract_features_from_ring(ring_pts)
            if e.size:
                all_edges.append(e)
            if p.size:
                all_planar.append(p)

        edge_cloud = np.vstack(all_edges) if all_edges else np.empty((0, 4), np.float32)
        planar_cloud = (
            np.vstack(all_planar) if all_planar else np.empty((0, 4), np.float32)
        )

        # Always publish, even if empty (width == 0)
        self.edge_publisher.publish(self.array_to_pointcloud2(edge_cloud, msg.header))
        self.planar_publisher.publish(
            self.array_to_pointcloud2(planar_cloud, msg.header)
        )

        total_in = int(pts.shape[0])
        n_edge = int(edge_cloud.shape[0])
        n_plan = int(planar_cloud.shape[0])
        reduction = 100.0 * (1.0 - (n_edge + n_plan) / max(1, total_in))

        self.get_logger().info(
            f"Features: input={total_in} -> edges={n_edge}, planar={n_plan} "
            f"({reduction:.1f}% reduction, rings={len(rings)})",
            throttle_duration_sec=1.0,
        )
        if n_plan == 0:
            self.get_logger().warn("Planar feature count is 0 for this frame.")


# -------------------------------------------------------------------------
# Entrypoint
# -------------------------------------------------------------------------
def main(args: Optional[List[str]] = None) -> None:
    """
    Node entry point.

    Initializes rclpy, creates and spins the LOAMFeatureExtractor node, and
    gracefully shuts down on interrupt.

    Args:
        args (Optional[List[str]]):
            Optional command-line arguments forwarded to rclpy.init().
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
