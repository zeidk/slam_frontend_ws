#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LOAM-Style Frame-to-Frame Scan Matcher

This module implements a point-to-plane Iterative Closest Point (ICP) algorithm
for incremental pose estimation in LiDAR-based SLAM. The node fuses edge and
planar features extracted from consecutive LiDAR scans and estimates the relative
transformation between frames using least-squares optimization in SE(3).

Key Features:
    - Point-to-plane ICP for robust registration
    - SE(3) exponential map for direct Lie algebra optimization
    - K-NN based surface normal estimation via PCA
    - Voxel-grid downsampling for efficiency
    - Publishes incremental odometry, trajectory, and TF transforms

Main Entry Point:
    :func:`main`

Main Class:
    :class:`ScanMatcherNode`
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
    """
    Convert a ROS 2 PointCloud2 message to an (N, 3) XYZ coordinate array.

    Extracts the x, y, z fields (assumed to be float32 at offsets 0, 4, 8 bytes)
    from each point in the cloud. Handles both organized and unorganized layouts.

    Parameters
    ----------
    msg : sensor_msgs.msg.PointCloud2
        Input point cloud message with standard x/y/z float32 layout.

    Returns
    -------
    np.ndarray
        Array of shape (N, 3) with dtype float32 containing point coordinates,
        or an empty (0, 3) array if the message is empty or has incompatible layout.

    Notes
    -----
    - Assumes point_step encodes the byte stride between consecutive points.
    - Does not validate field descriptors; assumes standard x/y/z layout.
    - Returns empty array on invalid input rather than raising exceptions.
    """
    n = msg.width * msg.height
    if n == 0 or msg.point_step < 12:
        return np.zeros((0, 3), dtype=np.float32)

    buf = memoryview(msg.data)
    arr = np.frombuffer(buf, dtype=np.uint8)

    xyz = np.empty((n, 3), dtype=np.float32)
    step = msg.point_step
    for i in range(n):
        base = i * step
        xyz[i, 0] = np.frombuffer(arr[base + 0: base + 4], dtype=np.float32, count=1)[0]
        xyz[i, 1] = np.frombuffer(arr[base + 4: base + 8], dtype=np.float32, count=1)[0]
        xyz[i, 2] = np.frombuffer(arr[base + 8: base + 12], dtype=np.float32, count=1)[0]
    return xyz


def _voxel_downsample(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    Downsample point cloud using voxel-grid hashing.

    Groups points into cubic voxels of side length `voxel_size` and retains
    one representative point per voxel (the first point encountered in sorted order).
    Improves ICP convergence speed and robustness by reducing point density.

    Parameters
    ----------
    xyz : np.ndarray
        Input point cloud of shape (N, 3) with dtype float32.
    voxel_size : float
        Side length of cubic voxel cells (meters). If ≤ 0 or None, returns input unchanged.

    Returns
    -------
    np.ndarray
        Downsampled cloud of shape (M, 3), where M ≤ N depends on spatial density.

    Notes
    -----
    - Uses NumPy unique() for hash-based grouping; deterministic but not preserving order.
    - Empty input or invalid voxel_size returns the input array.

    Examples
    --------
    >>> xyz = np.random.randn(1000, 3)
    >>> downsampled = _voxel_downsample(xyz, voxel_size=0.1)
    >>> downsampled.shape[0] < 1000
    True
    """
    if voxel_size is None or voxel_size <= 0.0 or xyz.size == 0:
        return xyz
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[np.sort(idx)]


def _pca_normal(points: np.ndarray) -> np.ndarray:
    """
    Estimate surface normal via Principal Component Analysis (PCA).

    Computes the covariance matrix of a point neighborhood and returns the
    eigenvector corresponding to the smallest eigenvalue, which approximates
    the surface normal.

    Parameters
    ----------
    points : np.ndarray
        Neighborhood points of shape (K, 3), where K ≥ 3.

    Returns
    -------
    np.ndarray
        Normal vector of shape (3,) (unit norm). Corresponds to the smallest
        eigenvalue of the covariance matrix.

    Notes
    -----
    - Assumes K ≥ 3 for meaningful covariance estimation.
    - Returns unit eigenvector without explicit normalization (eigenvectors from
      np.linalg.eigh are already normalized).
    - Does not handle or orient the normal; caller may need to ensure consistent
      orientation (e.g., toward sensor or global up).

    Mathematical Definition
    ----------------------
    Given points :math:`\\{p_1, \\ldots, p_K\\}`:

    .. math::

        \\bar{p} &= \\frac{1}{K} \\sum_{i=1}^K p_i \\\\
        C &= \\frac{1}{K-1} \\sum_{i=1}^K (p_i - \\bar{p})(p_i - \\bar{p})^T \\\\
        \\mathbf{n} &= \\arg\\min_{\\|\\mathbf{v}\\|=1} \\mathbf{v}^T C \\mathbf{v}
    """
    c = points.mean(axis=0)
    Q = points - c
    C = Q.T @ Q / max(len(points) - 1, 1)
    w, v = np.linalg.eigh(C)
    return v[:, 0]


def _estimate_normals(xyz_ref: np.ndarray, k: int = 20) -> Tuple[np.ndarray, cKDTree]:
    """
    Estimate per-point surface normals on a reference cloud using k-NN PCA.

    For each point, performs k-nearest neighbor search and estimates the surface
    normal of the local neighborhood via PCA. Normals are oriented consistently
    (toward +Z as a heuristic for typical ground-based LiDAR).

    Parameters
    ----------
    xyz_ref : np.ndarray
        Reference point cloud of shape (N, 3) with dtype float32.
    k : int, optional
        Number of neighbors for PCA-based normal estimation (default: 20).
        Clamped to the number of available points if insufficient.

    Returns
    -------
    normals : np.ndarray
        Normal vectors of shape (N, 3), dtype float32. Unit-norm vectors
        pointing toward +Z if possible.
    kdtree : scipy.spatial.cKDTree
        Pre-built KD-tree on xyz_ref for efficient neighbor queries in ICP.

    Notes
    -----
    - If a neighborhood has fewer than 3 points, a zero normal is returned.
    - Normals are oriented by checking the sign of the Z component and flipping
      if necessary for consistency.
    - Returns zero normals and a valid tree if xyz_ref has too few points.

    Raises
    ------
    ValueError
        If xyz_ref has shape other than (N, 3).
    """
    if xyz_ref.shape[0] < k + 1:
        return np.zeros_like(xyz_ref), cKDTree(xyz_ref)
    tree = cKDTree(xyz_ref)
    normals = np.zeros_like(xyz_ref)
    dists, idxs = tree.query(xyz_ref, k=min(k, xyz_ref.shape[0]))
    for i in range(xyz_ref.shape[0]):
        nbrs = xyz_ref[idxs[i]] if np.ndim(idxs[i]) > 0 else xyz_ref[[idxs[i]]]
        if nbrs.shape[0] >= 3:
            n = _pca_normal(nbrs)
            if n[2] < 0:
                n = -n
            normals[i] = n
    return normals, tree


def _skew(v: np.ndarray) -> np.ndarray:
    """
    Compute the skew-symmetric (cross-product) matrix for a 3D vector.

    Given a vector :math:`\mathbf{v} = [v_1, v_2, v_3]^T`, returns the matrix
    :math:`[\mathbf{v}]_\\times` such that :math:`[\mathbf{v}]_\\times \mathbf{u} = \mathbf{v} \\times \mathbf{u}`.

    Parameters
    ----------
    v : np.ndarray
        3D vector of shape (3,).

    Returns
    -------
    np.ndarray
        Skew-symmetric matrix of shape (3, 3), dtype float64.

    Notes
    -----
    The skew matrix is used in Lie algebra computations for SE(3) exponential maps.

    Mathematical Definition
    ----------------------

    .. math::

        [\\mathbf{v}]_{\\times} = \\begin{pmatrix}
            0 & -v_3 & v_2 \\\\
            v_3 & 0 & -v_1 \\\\
            -v_2 & v_1 & 0
        \\end{pmatrix}
    """
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]], dtype=np.float64)


def _se3_exp(xi: np.ndarray) -> np.ndarray:
    """
    Compute the SE(3) exponential map from a twist (Lie algebra element).

    Converts a 6D twist vector :math:`\\boldsymbol{\\xi} = [\\boldsymbol{\\omega}, \\mathbf{t}]^T`
    (angular velocity and translational velocity) into an SE(3) matrix (4×4 homogeneous
    transformation). This is the core of direct Lie algebra optimization.

    Parameters
    ----------
    xi : np.ndarray
        Twist vector of shape (6,) containing:
        - xi[0:3]: angular velocity :math:`\\boldsymbol{\\omega}` (rad)
        - xi[3:6]: translational velocity :math:`\\mathbf{t}` (m)

    Returns
    -------
    np.ndarray
        SE(3) transformation matrix of shape (4, 4), dtype float64.
        Bottom row is [0, 0, 0, 1].

    Notes
    -----
    - Uses Rodrigues' rotation formula for the exponential rotation.
    - Returns identity transform if :math:`\\|\\boldsymbol{\\omega}\\| < 10^{-12}`.
    - Output is used in ICP iterations as incremental transforms.

    Mathematical Definition
    ----------------------
    Given twist :math:`\\boldsymbol{\\xi} = [\\boldsymbol{\\omega}, \\mathbf{t}]^T`:

    .. math::

        \\exp(\\boldsymbol{\\xi}) &= \\begin{pmatrix} R & V \\mathbf{t} \\\\ 0 & 1 \\end{pmatrix} \\\\
        R &= I + \\sin(\\theta) [\\boldsymbol{\\omega}]_{\\times} + (1 - \\cos(\\theta)) [\\boldsymbol{\\omega}]_{\\times}^2 \\\\
        V &= I + \\frac{1-\\cos(\\theta)}{\\theta} [\\boldsymbol{\\omega}]_{\\times} + \\frac{\\theta - \\sin(\\theta)}{\\theta^2} [\\boldsymbol{\\omega}]_{\\times}^2

    where :math:`\\theta = \\|\\boldsymbol{\\omega}\\|`.
    """
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
    """
    Apply a 4×4 SE(3) transformation to a set of 3D points.

    Parameters
    ----------
    T : np.ndarray
        4×4 homogeneous transformation matrix of shape (4, 4).
    pts : np.ndarray
        Point cloud of shape (N, 3).

    Returns
    -------
    np.ndarray
        Transformed points of shape (N, 3).

    Notes
    -----
    Efficiently batches the transformation as matrix operations rather than
    point-by-point loops.

    Mathematical Definition
    ----------------------

    .. math::

        \\mathbf{p}'_i = R \\mathbf{p}_i + \\mathbf{t}
    """
    return (T[:3, :3] @ pts.T + T[:3, 3:4]).T


class ScanMatcherNode(Node):
    """
    ROS 2 node implementing frame-to-frame scan matching via point-to-plane ICP.

    This node subscribes to edge and planar feature point clouds from consecutive
    LiDAR scans, performs incremental pose estimation using point-to-plane ICP,
    and publishes the integrated odometry, trajectory, and TF transforms.

    The algorithm:

    1. Fuses edge and planar features from current frame
    2. Voxel-downsamples the fused point cloud
    3. Runs iterative point-to-plane ICP against the previous frame
    4. Integrates the relative transformation into global pose
    5. Updates reference frame and publishes outputs

    Parameters (ROS 2)
    ------------------
    edge_topic : str
        Topic name for edge feature clouds (default: 'features/edge_cloud')
    planar_topic : str
        Topic name for planar feature clouds (default: 'features/planar_cloud')
    voxel_size : float
        Voxel-grid downsampling cell size in meters (default: 0.2)
    max_icp_iter : int
        Maximum number of ICP iterations per frame (default: 20)
    icp_dist_thresh : float
        Distance threshold for point-to-plane correspondences (meters, default: 1.0)
    knn_normals : int
        Number of neighbors for normal estimation (default: 20)
    frame_map : str
        ROS frame_id for global map frame (default: 'map')
    frame_base : str
        ROS frame_id for robot base link (default: 'base_link')
    odom_topic : str
        Topic name for publishing odometry (default: 'scan_match/odometry')
    trajectory_topic : str
        Topic name for publishing trajectory (default: 'scan_match/trajectory')

    Subscriptions
    -------------
    edge_topic : PointCloud2
        Edge feature clouds (high curvature points)
    planar_topic : PointCloud2
        Planar feature clouds (low curvature points)

    Publications
    -------------
    odom_topic : Odometry
        Odometry message with pose in map frame
    trajectory_topic : Path
        Accumulated trajectory of poses over time
    (TF) map → base_link : TransformStamped
        Broadcast pose transform from map to base_link

    Attributes
    ----------
    T_wb : np.ndarray
        Current world-to-base transformation (4×4 SE(3) matrix)
    _last_xyz : Optional[np.ndarray]
        Previous frame's point cloud (used as reference)
    _last_normals : Optional[np.ndarray]
        Pre-computed normals for previous frame
    _last_tree : Optional[cKDTree]
        KD-tree on previous frame for fast correspondence search
    """

    def __init__(self) -> None:
        """
        Initialize the scan matcher node.

        Declares ROS parameters with defaults, initializes publishers/subscribers,
        sets up internal state, and logs configuration.
        """
        super().__init__('loam_scan_matcher')

        # Parameters
        self.declare_parameter('edge_topic', 'features/edge_cloud')
        self.declare_parameter('planar_topic', 'features/planar_cloud')
        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('max_icp_iter', 20)
        self.declare_parameter('icp_dist_thresh', 1.0)
        self.declare_parameter('knn_normals', 20)
        self.declare_parameter('frame_map', 'map')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('odom_topic', 'scan_match/odometry')
        self.declare_parameter('trajectory_topic', 'scan_match/trajectory')

        self.edge_topic = self.get_parameter('edge_topic').value
        self.planar_topic = self.get_parameter('planar_topic').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_icp_iter = int(self.get_parameter('max_icp_iter').value)
        self.icp_dist_thresh = float(self.get_parameter('icp_dist_thresh').value)
        self.knn_normals = int(self.get_parameter('knn_normals').value)
        self.frame_map = self.get_parameter('frame_map').value
        self.frame_base = self.get_parameter('frame_base').value
        odom_topic = self.get_parameter('odom_topic').value
        trajectory_topic = self.get_parameter('trajectory_topic').value

        # State
        self._last_xyz: Optional[np.ndarray] = None
        self._last_normals: Optional[np.ndarray] = None
        self._last_tree: Optional[cKDTree] = None

        self.T_wb = np.eye(4, dtype=np.float64)
        self._path = Path()
        self._path.header.frame_id = self.frame_map

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._path_pub = self.create_publisher(Path, trajectory_topic, 10)
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
            f"Frame: {self.frame_map} -> {self.frame_base}\n"
            f"Params: voxel={self.voxel_size} m, iters={self.max_icp_iter}, "
            f"dist={self.icp_dist_thresh} m, k={self.knn_normals}"
        )

    def _edge_cb(self, msg: PointCloud2) -> None:
        """
        Callback for edge feature cloud messages.

        Stores the latest edge features for fusion with the next planar message.
        Edge features have high curvature and represent sharp geometric features.

        Parameters
        ----------
        msg : PointCloud2
            Edge feature cloud from the current scan.
        """
        self._edge_xyz = _pc2_to_xyz(msg)
        self._edge_stamp = Time.from_msg(msg.header.stamp)

    def _planar_cb(self, msg: PointCloud2) -> None:
        """
        Callback for planar feature cloud messages.

        Triggered when planar features arrive. Fuses latest edge + current planar
        features, performs voxel downsampling, and triggers the scan matching pipeline.

        Parameters
        ----------
        msg : PointCloud2
            Planar feature cloud from the current scan (low-curvature points).
        """
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
        """
        Process a single frame: perform ICP and update global pose.

        On the first frame, initializes the reference. On subsequent frames,
        runs point-to-plane ICP to estimate the relative transformation, integrates
        it into the global pose, and publishes odometry/trajectory/TF.

        Parameters
        ----------
        xyz_curr : np.ndarray
            Current frame's downsampled point cloud (N, 3).
        stamp : rclpy.time.Time
            Timestamp of the current scan.

        Notes
        -----
        - First frame with < 20 points is skipped; normal processing begins at frame 2.
        - ICP runs for at most `max_icp_iter` iterations, with early stopping if
          the update norm falls below 1e-6.
        """
        if xyz_curr.size == 0:
            return

        if self._last_xyz is None or self._last_xyz.shape[0] < 20:
            self._last_xyz = xyz_curr
            self._last_normals, self._last_tree = _estimate_normals(self._last_xyz, self.knn_normals)
            return

        T = np.eye(4, dtype=np.float64)
        curr = xyz_curr.copy()

        for _ in range(self.max_icp_iter):
            curr_tf = _transform_pts(T, curr)

            dists, idxs = self._last_tree.query(curr_tf, k=1)
            mask = dists < self.icp_dist_thresh
            if not np.any(mask):
                break

            q = curr_tf[mask]
            p = self._last_xyz[idxs[mask]]
            n = self._last_normals[idxs[mask]]

            cq = np.cross(q, n)
            A = np.hstack([cq, n]).astype(np.float64)
            b = np.einsum('ij,ij->i', n, (p - q)).astype(np.float64)

            AtA = A.T @ A + 1e-6 * np.eye(6)
            Atb = A.T @ b
            try:
                xi = np.linalg.solve(AtA, Atb)
            except np.linalg.LinAlgError:
                break

            dT = _se3_exp(xi)
            T = dT @ T

            if np.linalg.norm(xi) < 1e-6:
                break

        self.T_wb = self.T_wb @ T

        self._last_xyz = xyz_curr
        self._last_normals, self._last_tree = _estimate_normals(self._last_xyz, self.knn_normals)

        self._publish_odometry_and_tf(stamp)
        self._append_and_publish_path(stamp)

    def _publish_odometry_and_tf(self, stamp) -> None:
        """
        Publish Odometry message and TF transform for the current pose.

        Publishes the estimated pose in the map frame using both a ROS Odometry
        message and a TF transform. The pose represents the robot's current position
        and orientation in the global map coordinate system.

        Parameters
        ----------
        stamp : rclpy.time.Time
            Timestamp for the published messages.

        Notes
        -----
        - Converts SE(3) matrix to quaternion using tf_transformations.
        - Publishes both nav_msgs/Odometry and geometry_msgs/TransformStamped.
        - Frame hierarchy: map (parent) -> base_link (child)
        """
        T = self.T_wb
        t = T[:3, 3]
        q = tft.quaternion_from_matrix(T)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_map
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
        tf.header.frame_id = self.frame_map
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
        """
        Append the current pose to the trajectory and publish.

        Accumulates poses over time to form a nav_msgs/Path message representing
        the robot's trajectory through the environment. All poses are expressed
        in the map frame.

        Parameters
        ----------
        stamp : rclpy.time.Time
            Timestamp for the pose.

        Notes
        -----
        The trajectory grows unbounded over the session; consider implementing
        trimming or serialization for long-running missions.
        """
        T = self.T_wb
        t = T[:3, 3]
        q = tft.quaternion_from_matrix(T)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_map
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
    """
    Entry point for the scan matcher node.

    Initializes the ROS 2 context, creates the ScanMatcherNode, and spins it
    (blocking until shutdown). Handles keyboard interrupts gracefully.

    Parameters
    ----------
    args : list, optional
        Command-line arguments to pass to rclpy.init().
    """
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