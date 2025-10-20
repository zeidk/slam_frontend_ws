#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Scan Matching Node (Point-to-Plane ICP, NumPy + SciPy).

This node estimates frame-to-frame motion from LOAM-style feature clouds
(edge + planar) and publishes:
    - nav_msgs/Odometry on ``/odom``
    - TF: ``odom -> base_link``
    - nav_msgs/Path on ``/trajectory`` for easy visualization in Foxglove Studio

Algorithm (minimal, educational):
    1. Merge current edge + planar features.
    2. Voxel downsample (NumPy grid-hash).
    3. Build KD-tree over *reference* (previous frame).
    4. Estimate *reference* normals via local PCA (k-NN).
    5. Run point-to-plane ICP (small-angle approx) for up to N iterations:
        - Transform current → reference
        - Find correspondences (nearest neighbor with distance threshold)
        - Solve least-squares for twist ξ = [ω, t]
        - Update transform (compose exponential map)
    6. Integrate ΔT into world pose and publish /odom, TF, and /trajectory.

Notes
-----
- This implementation favors clarity over ultimate robustness/performance.
- For production, consider outlier rejection, robust loss, and a local map
  (sliding window) rather than pure prev-frame matching.

Parameters (declared)
---------------------
edge_topic : str
    Input edge features topic (PointCloud2). Default: ``features/edge_cloud``.
planar_topic : str
    Input planar features topic (PointCloud2). Default: ``features/planar_cloud``.
voxel_size : float
    Voxel size for downsampling (meters). Default: 0.2.
max_icp_iter : int
    Maximum ICP iterations per frame. Default: 20.
icp_dist_thresh : float
    Max correspondence distance (meters). Default: 1.0.
knn_normals : int
    k-NN for normal estimation on reference. Default: 20.
frame_odom : str
    World/odometry frame. Default: ``odom``.
frame_base : str
    Robot base frame. Default: ``base_link``.

"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf_transformations as tft
import tf2_ros


def _pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 to (N, 3) float32 XYZ array.

    Assumes a common layout where x/y/z are float32 at offsets 0/4/8.
    Returns an empty array if layout is incompatible.
    """
    n = msg.width * msg.height
    if n == 0 or msg.point_step < 12:
        return np.zeros((0, 3), dtype=np.float32)

    buf = memoryview(msg.data)
    arr = np.frombuffer(buf, dtype=np.uint8)

    # Extract x, y, z by reading 4-byte floats at the right offsets per point
    xyz = np.empty((n, 3), dtype=np.float32)
    step = msg.point_step
    for i in range(n):
        base = i * step
        xyz[i, 0] = np.frombuffer(arr[base + 0: base + 4], dtype=np.float32, count=1)[0]
        xyz[i, 1] = np.frombuffer(arr[base + 4: base + 8], dtype=np.float32, count=1)[0]
        xyz[i, 2] = np.frombuffer(arr[base + 8: base + 12], dtype=np.float32, count=1)[0]
    return xyz


def _voxel_downsample(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    """Voxel-grid downsample using NumPy hashing.

    Parameters
    ----------
    xyz : (N, 3) array
    voxel_size : float

    Returns
    -------
    (M, 3) array
        Representative point per voxel (first seen).
    """
    if voxel_size is None or voxel_size <= 0.0 or xyz.size == 0:
        return xyz
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    # hash as tuple of ints
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[np.sort(idx)]


def _pca_normal(points: np.ndarray) -> np.ndarray:
    """Estimate normal via PCA: eigenvector of smallest eigenvalue."""
    c = points.mean(axis=0)
    Q = points - c
    C = Q.T @ Q / max(len(points) - 1, 1)
    w, v = np.linalg.eigh(C)
    return v[:, 0]  # smallest eigenvalue


def _estimate_normals(xyz_ref: np.ndarray, k: int = 20) -> Tuple[np.ndarray, cKDTree]:
    """Compute per-point normals on reference cloud using k-NN PCA.

    Returns
    -------
    normals : (N, 3) array
    kdtree : scipy.spatial.cKDTree
        KD-tree built on xyz_ref.
    """
    if xyz_ref.shape[0] < k + 1:
        return np.zeros_like(xyz_ref), cKDTree(xyz_ref)
    tree = cKDTree(xyz_ref)
    normals = np.zeros_like(xyz_ref)
    # Query k neighbors (including self)
    dists, idxs = tree.query(xyz_ref, k=min(k, xyz_ref.shape[0]))
    for i in range(xyz_ref.shape[0]):
        nbrs = xyz_ref[idxs[i]] if np.ndim(idxs[i]) > 0 else xyz_ref[[idxs[i]]]
        if nbrs.shape[0] >= 3:
            n = _pca_normal(nbrs)
            # Orient normals consistently (toward +Z as a heuristic)
            if n[2] < 0:
                n = -n
            normals[i] = n
    return normals, tree


def _skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix [v]_x for cross products."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]], dtype=np.float64)


def _se3_exp(xi: np.ndarray) -> np.ndarray:
    """SE(3) exponential map from twist xi = [ω(3), t(3)]."""
    w = xi[:3]
    t = xi[3:]
    theta = np.linalg.norm(w)
    R = np.eye(3)
    V = np.eye(3)
    if theta > 1e-12:
        wn = w / theta
        wx = _skew(wn)
        s = np.sin(theta)
        c = np.cos(theta)
        R = np.eye(3) + s * wx + (1 - c) * (wx @ wx)
        V = (np.eye(3) +
             (1 - c) / theta * wx +
             (theta - s) / (theta**2) * (wx @ wx))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (V @ t).astype(np.float64)
    return T


def _transform_pts(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply 4x4 transform to (N,3) points."""
    return (T[:3, :3] @ pts.T + T[:3, 3:4]).T


class ScanMatcherNode(Node):
    """Frame-to-frame scan matcher using point-to-plane ICP (NumPy + SciPy).

    Subscribes to two feature topics, estimates ΔT between consecutive frames,
    integrates into a global pose ``T_wb``, and publishes odometry, TF, and path.
    """

    def __init__(self) -> None:
        """Initialize node, parameters, pubs/subs, and state."""
        super().__init__('loam_scan_matcher')

        # Parameters
        self.declare_parameter('edge_topic', 'features/edge_cloud')
        self.declare_parameter('planar_topic', 'features/planar_cloud')
        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('max_icp_iter', 20)
        self.declare_parameter('icp_dist_thresh', 1.0)
        self.declare_parameter('knn_normals', 20)
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')

        self.edge_topic = self.get_parameter('edge_topic').value
        self.planar_topic = self.get_parameter('planar_topic').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_icp_iter = int(self.get_parameter('max_icp_iter').value)
        self.icp_dist_thresh = float(self.get_parameter('icp_dist_thresh').value)
        self.knn_normals = int(self.get_parameter('knn_normals').value)
        self.frame_odom = self.get_parameter('frame_odom').value
        self.frame_base = self.get_parameter('frame_base').value

        # State
        self._last_xyz: Optional[np.ndarray] = None
        self._last_normals: Optional[np.ndarray] = None
        self._last_tree: Optional[cKDTree] = None

        self.T_wb = np.eye(4, dtype=np.float64)
        self._path = Path()
        self._path.header.frame_id = self.frame_odom

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._path_pub = self.create_publisher(Path, 'trajectory', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        self._edge_xyz: Optional[np.ndarray] = None
        self._edge_stamp: Optional[Time] = None
        self._planar_xyz: Optional[np.ndarray] = None
        self._planar_stamp: Optional[Time] = None

        self.create_subscription(PointCloud2, self.edge_topic, self._edge_cb, 10)
        self.create_subscription(PointCloud2, self.planar_topic, self._planar_cb, 10)

        self.get_logger().info(
            f"Scan matcher ready. Subscribing to:\n"
            f"  - {self.edge_topic}\n"
            f"  - {self.planar_topic}\n"
            f"Params: voxel={self.voxel_size} m, iters={self.max_icp_iter}, "
            f"dist={self.icp_dist_thresh} m, k={self.knn_normals}"
        )

    def _edge_cb(self, msg: PointCloud2) -> None:
        """Store latest edge features."""
        self._edge_xyz = _pc2_to_xyz(msg)
        self._edge_stamp = Time.from_msg(msg.header.stamp)

    def _planar_cb(self, msg: PointCloud2) -> None:
        """Fuse latest edge + current planar, then run a scan-matching step."""
        self._planar_xyz = _pc2_to_xyz(msg)
        self._planar_stamp = Time.from_msg(msg.header.stamp)

        if self._planar_xyz is None or self._planar_xyz.size == 0:
            return

        if self._edge_xyz is not None and self._edge_xyz.size > 0:
            xyz = np.vstack([self._edge_xyz, self._planar_xyz])
        else:
            xyz = self._planar_xyz

        xyz = _voxel_downsample(xyz, self.voxel_size)
        self._process_frame(xyz, msg.header.stamp)

    def _process_frame(self, xyz_curr: np.ndarray, stamp) -> None:
        """Perform point-to-plane ICP against previous frame and publish outputs."""
        if xyz_curr.size == 0:
            return

        # Initialize reference on first frame
        if self._last_xyz is None or self._last_xyz.shape[0] < 20:
            self._last_xyz = xyz_curr
            self._last_normals, self._last_tree = _estimate_normals(self._last_xyz, self.knn_normals)
            return

        # Current-to-reference registration (prev frame as reference)
        T = np.eye(4, dtype=np.float64)
        curr = xyz_curr.copy()

        for _ in range(self.max_icp_iter):
            # Transform current by the running estimate
            curr_tf = _transform_pts(T, curr)

            # Find correspondences in reference
            dists, idxs = self._last_tree.query(curr_tf, k=1)
            mask = dists < self.icp_dist_thresh
            if not np.any(mask):
                break

            q = curr_tf[mask]                 # transformed current points
            p = self._last_xyz[idxs[mask]]    # reference points
            n = self._last_normals[idxs[mask]]  # reference normals

            # Build linear system A xi = b   (point-to-plane)
            # n^T( (R q + t) - p ) ≈ n^T( q + ω×q + t - p )
            # A_i = [ (q × n)^T   n^T ] , b_i = n^T (p - q)
            cq = np.cross(q, n)
            A = np.hstack([cq, n]).astype(np.float64)
            b = np.einsum('ij,ij->i', n, (p - q)).astype(np.float64)

            # Least-squares (with tiny Tikhonov)
            AtA = A.T @ A + 1e-6 * np.eye(6)
            Atb = A.T @ b
            try:
                xi = np.linalg.solve(AtA, Atb)
            except np.linalg.LinAlgError:
                break

            dT = _se3_exp(xi)
            T = dT @ T

            # Early stopping if small update
            if np.linalg.norm(xi) < 1e-6:
                break

        # Update world pose (prev -> curr)
        self.T_wb = self.T_wb @ T

        # Update reference with current frame
        self._last_xyz = xyz_curr
        self._last_normals, self._last_tree = _estimate_normals(self._last_xyz, self.knn_normals)

        # Publish
        self._publish_odometry_and_tf(stamp)
        self._append_and_publish_path(stamp)

    def _publish_odometry_and_tf(self, stamp) -> None:
        """Publish nav_msgs/Odometry and TF from current pose T_wb."""
        T = self.T_wb
        t = T[:3, 3]
        q = tft.quaternion_from_matrix(T)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = float(t[0])
        odom.pose.pose.position.y = float(t[1])
        odom.pose.pose.position.z = float(t[2])
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_odom
        tf.child_frame_id = self.frame_base
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])
        self._tf_broadcaster.sendTransform(tf)

    def _append_and_publish_path(self, stamp) -> None:
        """Append the current pose to /trajectory and publish."""
        T = self.T_wb
        t = T[:3, 3]
        q = tft.quaternion_from_matrix(T)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_odom
        pose.pose.position.x = float(t[0])
        pose.pose.position.y = float(t[1])
        pose.pose.position.z = float(t[2])
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])

        self._path.header.stamp = stamp
        self._path.poses.append(pose)
        self._path_pub.publish(self._path)


def main(args=None) -> None:
    """Entry point: initialize ROS 2, spin node, and shutdown cleanly."""
    rclpy.init(args=args)
    node = ScanMatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
