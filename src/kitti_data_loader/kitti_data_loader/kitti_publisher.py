#!/usr/bin/env python3
"""
kitti_publisher.py
========================

ROS 2 node that publishes KITTI Odometry LiDAR scans and ground-truth poses with **correct TF semantics**,
sensor QoS for point clouds, and robust file/directory checks.

Naming convention
-----------------
All **class data members** use a **leading underscore** to indicate internal attributes (e.g., ``self._seq``, ``self._scan_files``).
This applies to variables initialized in ``__init__`` such as frames, topics, paths, publishers, and TF broadcasters.

Key behavior
------------
**Coordinate system conversion (CRITICAL):**
- KITTI Camera (cam0): x=right, y=down, z=forward
- KITTI Velodyne: x=forward, y=left, z=up (ALREADY MATCHES ROS REP-103!)
- ROS REP-103: x=forward, y=left, z=up

The rotation R_K2R is ONLY applied to:
1. Camera poses from poses/<SEQ>.txt (converting cam0 frame to ROS base_link frame)
2. The Tr_velo_to_cam calibration matrix (which is expressed in camera coordinates)

The Velodyne point cloud data is NOT transformed because it's already in ROS-compatible coordinates!

1) Publishes the **global** vehicle pose as a ``map -> base_link`` transform on every scan (and also as ``PoseStamped``).
2) Publishes a **static** ``base_link -> velodyne`` transform once, using KITTI calibration if available.
3) Adds **robust directory / file existence checks** with clear errors.
4) Uses **sensor QoS** (``qos_profile_sensor_data``) for point cloud publishing to reduce drops under load.

Assumptions
-----------
- Directory layout follows KITTI Odometry:

  .. code-block:: text

    <kitti_data_dir>/
      data_odometry_velodyne/dataset/sequences/<SEQ>/velodyne/*.bin
      data_odometry_poses/dataset/poses/<SEQ>.txt             (optional; required for map->base_link)
      data_odometry_calib/dataset/sequences/<SEQ>/calib.txt   (or calib_<SEQ>.txt; used for base_link->velodyne)

Outputs
-------
- sensor_msgs/PointCloud2 on ``pointcloud_topic``
- geometry_msgs/PoseStamped on ``ground_truth_pose_topic``
- TF:   map -> base_link (dynamic) each tick
- TF:   base_link -> velodyne (static) once from calibration if found (identity otherwise)

Usage
-----
.. code-block:: bash

    ros2 launch kitti_data_loader kitti_data_loader.launch.py

"""
from __future__ import annotations

import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header

import tf_transformations as tft
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# ---------------------------- Coordinate frame conversion ----------------------------
# KITTI camera frame (cam0): x=right, y=down, z=forward
# KITTI Velodyne frame: x=forward, y=left, z=up (ALREADY ROS-compatible!)
# ROS REP-103: x=forward, y=left, z=up
# 
# Rotation matrix to convert CAMERA poses from KITTI cam0 to ROS base_link:
# ROS_x = KITTI_z  (forward)
# ROS_y = -KITTI_x (left) 
# ROS_z = -KITTI_y (up)
R_CAM_K2R = np.array([[ 0,  0,  1],
                      [-1,  0,  0],
                      [ 0, -1,  0]], dtype=np.float64)
R4_CAM = np.eye(4, dtype=np.float64); R4_CAM[:3,:3] = R_CAM_K2R
R4_CAM_inv = R4_CAM.T

# ---------------------------- Utilities ----------------------------

def read_kitti_poses_txt(path: str) -> np.ndarray:
    """
    Read KITTI odometry pose file (poses/<SEQ>.txt).

    Each line has 12 numbers (row-major 3x4 matrix) representing the camera0 pose
    of the vehicle in the global coordinate system for each timestamp.

    Returns
    -------
    poses : (N, 4, 4) float64
        Homogeneous transforms ``T_map_cam0`` for each frame in KITTI camera coordinates.
    """
    mats = []
    with open(path, "r") as f:
        for line in f:
            vals = [float(x) for x in line.strip().split()]
            if len(vals) != 12:
                continue
            T = np.eye(4, dtype=np.float64)
            T[:3, :4] = np.array(vals, dtype=np.float64).reshape(3, 4)
            mats.append(T)
    return np.stack(mats, axis=0) if mats else np.zeros((0,4,4), dtype=np.float64)


def load_calibration_matrix(calib_file: str) -> dict:
    """
    Parse a KITTI calibration file into a dict of named 3x4 (or 3x3) matrices.

    The file may contain keys like:
        P0, P1, P2, P3, Tr (velo->cam0), Tr_velo_to_cam, Tr_imu_to_velo, etc.

    Returns
    -------
    dict[str, np.ndarray]
        Keys to (3,4) float64 matrices (3x3 values are padded to 3x4 with zeros).
    """
    out = {}
    with open(calib_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or ":" not in line:
                continue
            key, rest = line.split(":", 1)
            vals = [float(x) for x in rest.strip().split()]
            if len(vals) in (12, 9):
                if len(vals) == 12:
                    M = np.array(vals, dtype=np.float64).reshape(3, 4)
                else:
                    # Some files may contain 3x3 (e.g., intrinsics). Pad if needed.
                    M = np.hstack([np.array(vals, dtype=np.float64).reshape(3,3), np.zeros((3,1))])
                out[key.strip()] = M
    return out


def homog_from_3x4(M34: np.ndarray) -> np.ndarray:
    """Convert 3x4 to 4x4 homogeneous pose."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :4] = M34
    return T


def inv_SE3(T: np.ndarray) -> np.ndarray:
    """Efficient inverse of an SE(3) transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def T_to_xyzw(T: np.ndarray):
    """Return position (x,y,z) and quaternion (x,y,z,w) from 4x4 T."""
    x, y, z = T[:3, 3]
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)
    return x, y, z, qx, qy, qz, qw


def find_calib_for_sequence(calib_root: str, seq: str) -> str | None:
    """
    Attempt to locate the calibration file for a given sequence.

    We try a few common layouts inside ``data_odometry_calib``:
      - dataset/sequences/<SEQ>/calib.txt
      - dataset/sequences/<SEQ>/calib_<SEQ>.txt
      - calib_<SEQ>.txt (at calib_root or immediate subdir)
    """
    candidates = [
        os.path.join(calib_root, "dataset", "sequences", seq, "calib.txt"),
        os.path.join(calib_root, "dataset", "sequences", seq, f"calib_{seq}.txt"),
        os.path.join(calib_root, f"calib_{seq}.txt"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    # fallback: search by name
    for root, _, files in os.walk(calib_root):
        for name in files:
            if seq in name and "calib" in name and name.endswith(".txt"):
                return os.path.join(root, name)
    return None


def make_pointcloud2(xyz_i: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """
    Create a sensor_msgs/PointCloud2 from Nx4 array [x,y,z,intensity].

    Parameters
    ----------
    xyz_i : (N, 4) float32
    frame_id : str
    stamp : builtin_interfaces/Time

    Returns
    -------
    PointCloud2
    """
    assert xyz_i.ndim == 2 and xyz_i.shape[1] in (3,4), "xyz_i must be Nx3 or Nx4"
    if xyz_i.shape[1] == 3:
        xyz_i = np.hstack([xyz_i, np.zeros((xyz_i.shape[0],1), dtype=xyz_i.dtype)])

    data = xyz_i.astype(np.float32).tobytes()
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = xyz_i.shape[0]
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = data
    return msg


# ---------------------------- Node ----------------------------

class KittiPublisher(Node):
    """
    Publish KITTI odometry LiDAR scans with correct TFs.

    This node:
      * streams .bin Velodyne scans (already in ROS-compatible coordinates)
      * publishes PoseStamped (map frame) using KITTI poses (converted from camera to ROS frame)
      * broadcasts TF: map->base_link (dynamic), base_link->velodyne (static from calib)

    Internal attributes
    -------------------
    Attributes are prefixed with a leading underscore, e.g.:
      - ``self._seq`` (sequence id, two-digit string)
      - ``self._rate_hz`` (publish rate)
      - ``self._pointcloud_topic``, ``self._pose_topic``
      - ``self._map_frame``, ``self._base_frame``, ``self._lidar_frame``
      - ``self._velodyne_path``, ``self._poses_file``, ``self._calib_root``
      - ``self._scan_files``, ``self._poses``
      - ``self._tf_broadcaster``, ``self._static_tf_broadcaster``
      - ``self._pc_pub``, ``self._pose_pub``
      - ``self._scan_index``, ``self._timer``
    """
    def __init__(self):
        super().__init__("kitti_publisher")

        # --- Parameters ---
        self.declare_parameter("kitti_data_dir", ".")
        self.declare_parameter("dataset_sequence", "00")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("pointcloud_topic", "/kitti/pointcloud_raw")
        self.declare_parameter("ground_truth_pose_topic", "/kitti/ground_truth_pose")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("lidar_frame_id", "velodyne")
        self.declare_parameter("base_equals_cam0", True)
        self.declare_parameter("use_cam0_poses", True)

        kitti_root = self.get_parameter("kitti_data_dir").get_parameter_value().string_value
        seq_in = self.get_parameter("dataset_sequence").get_parameter_value().string_value
        # Strip 's' or 'S' prefix if present, then zero-pad to 2 digits
        self._seq = seq_in[1:] if seq_in.startswith(("s", "S")) else seq_in
        self._seq = self._seq.zfill(2)
        
        self._rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self._pose_topic = self.get_parameter("ground_truth_pose_topic").value
        self._map_frame = self.get_parameter("map_frame_id").value
        self._base_frame = self.get_parameter("base_frame_id").value
        self._lidar_frame = self.get_parameter("lidar_frame_id").value
        self._base_equals_cam0 = bool(self.get_parameter("base_equals_cam0").value)
        self._use_cam0_poses = bool(self.get_parameter("use_cam0_poses").value)

        # --- Paths ---
        self._velodyne_path = os.path.join(
            kitti_root, "data_odometry_velodyne", "dataset", "sequences", self._seq, "velodyne"
        )
        self._poses_file = os.path.join(
            kitti_root, "data_odometry_poses", "dataset", "poses", f"{self._seq}.txt"
        )
        self._calib_root = os.path.join(kitti_root, "data_odometry_calib")

        # --- Robust existence checks ---
        if not os.path.isdir(self._velodyne_path):
            self.get_logger().error(f"Velodyne directory not found: {self._velodyne_path}")
            raise SystemExit(1)

        if self._use_cam0_poses and not os.path.isfile(self._poses_file):
            self.get_logger().error(
                f"Poses file not found (required with use_cam0_poses=True): {self._poses_file}"
            )
            raise SystemExit(1)
        
        if not os.path.isdir(self._calib_root):
            self.get_logger().warn(
                f"Calibration root not found: {self._calib_root} (will assume identity base->velodyne)"
            )

        # --- Load data lists ---
        self._scan_files = sorted([f for f in os.listdir(self._velodyne_path) if f.endswith(".bin")])
        if not self._scan_files:
            self.get_logger().error(f"No .bin scans found in {self._velodyne_path}")
            raise SystemExit(1)

        self._poses = read_kitti_poses_txt(self._poses_file) if self._use_cam0_poses else np.zeros((0,4,4))
        if self._use_cam0_poses and self._poses.shape[0] == 0:
            self.get_logger().error(f"Pose file parsed but empty: {self._poses_file}")
            raise SystemExit(1)

        # --- TF broadcasters ---
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        # --- Publish static base_link->velodyne from calibration ---
        self._publish_static_base_to_velodyne_from_calib(kitti_root)

        # --- Publishers ---
        self._pc_pub = self.create_publisher(PointCloud2, self._pointcloud_topic, qos_profile_sensor_data)
        self._pose_pub = self.create_publisher(PoseStamped, self._pose_topic, 10)

        # --- Timer ---
        period = 1.0 / max(1e-6, self._rate_hz)
        self._scan_index = 0
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"Publishing KITTI sequence {self._seq} at {self._rate_hz:.2f} Hz")
        self.get_logger().info(f"Frames: map={self._map_frame}, base={self._base_frame}, lidar={self._lidar_frame}")
        self.get_logger().info(f"Scans: {len(self._scan_files)}; Poses: {self._poses.shape[0]}")

    # ---------------------------- Calibration & Static TF ----------------------------

    def _publish_static_base_to_velodyne_from_calib(self, kitti_root: str) -> None:
        """
        Compute and publish a static base_link->velodyne transform using KITTI calibration if available.
        
        CRITICAL: The Tr_velo_to_cam matrix relates:
        - Velodyne frame (x forward, y left, z up) 
        - to Camera frame (x right, y down, z forward)
        
        We need to convert this relationship to ROS frames where base_link is in ROS coordinates.
        """
        T_base_to_velo_ros = np.eye(4, dtype=np.float64)

        calib_file = find_calib_for_sequence(self._calib_root, self._seq) if os.path.isdir(self._calib_root) else None
        if calib_file and os.path.isfile(calib_file):
            try:
                mats = load_calibration_matrix(calib_file)
                if self._base_equals_cam0 and ("Tr_velo_to_cam" in mats or "Tr" in mats):
                    M = mats.get("Tr_velo_to_cam", mats.get("Tr"))
                    # This is T_velo_to_cam0 in KITTI native coordinates
                    # Velodyne: x fwd, y left, z up
                    # Cam0: x right, y down, z fwd
                    T_velo_to_cam0_kitti = homog_from_3x4(M)
                    
                    # We need T_base(ROS)_to_velo(ROS)
                    # Since velo is already ROS-like, and base should be cam0 converted to ROS:
                    # T_base(ROS)_to_velo(ROS) = R_CAM @ inv(T_velo_to_cam0) @ R_CAM_inv
                    # Actually simpler: base_ROS = R_CAM @ cam0_KITTI @ R_CAM_inv
                    #                   velo_ROS = velo_KITTI (no change)
                    # So: T_base(ROS)_to_velo(ROS) = R_CAM @ cam0_to_velo_KITTI @ R_CAM_inv
                    #                                = R_CAM @ inv(T_velo_to_cam0) @ R_CAM_inv
                    
                    # @TODO issue here
                    # T_cam0_to_velo_kitti = inv_SE3(T_velo_to_cam0_kitti)
                    # T_base_to_velo_ros = R4_CAM @ T_cam0_to_velo_kitti @ R4_CAM_inv
                    
                    T_base_to_velo_ros = R4_CAM @ T_velo_to_cam0_kitti
                    
                    self.get_logger().info(
                        f"Static TF base(cam0)->velodyne from {os.path.basename(calib_file)} "
                        f"(cam frame converted to ROS, velo unchanged)"
                    )
                elif (not self._base_equals_cam0) and ("Tr_imu_to_velo" in mats):
                    # If base is IMU, the transform is already between two sensor frames
                    # Need to check IMU coordinate convention (often same as velodyne or needs conversion)
                    T_imu_to_velo_kitti = homog_from_3x4(mats["Tr_imu_to_velo"])
                    # Assuming IMU frame is also ROS-like (common in KITTI)
                    T_base_to_velo_ros = T_imu_to_velo_kitti
                    self.get_logger().info(
                        f"Static TF base(imu)->velodyne from {os.path.basename(calib_file)} "
                        f"(assuming both IMU and velo are ROS-compatible)"
                    )
                else:
                    self.get_logger().warn(
                        "Calibration found, but no suitable key (Tr_velo_to_cam/Tr or Tr_imu_to_velo). Using identity."
                    )
            except Exception as e:
                self.get_logger().warn(f"Failed to parse calibration ({calib_file}): {e}. Using identity.")
        else:
            self.get_logger().warn("No calibration file found for this sequence. Using identity base->velodyne.")

        x, y, z, qx, qy, qz, qw = T_to_xyzw(T_base_to_velo_ros)
        tfs = TransformStamped()
        tfs.header.frame_id = self._base_frame
        tfs.child_frame_id = self._lidar_frame
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.transform.translation.x = float(x)
        tfs.transform.translation.y = float(y)
        tfs.transform.translation.z = float(z)
        tfs.transform.rotation.x = float(qx)
        tfs.transform.rotation.y = float(qy)
        tfs.transform.rotation.z = float(qz)
        tfs.transform.rotation.w = float(qw)

        self._static_tf_broadcaster.sendTransform(tfs)

    # ---------------------------- Timer callback ----------------------------

    def _on_timer(self) -> None:
        if self._scan_index >= len(self._scan_files):
            self.get_logger().info("All KITTI scans have been published. Shutting down.")
            self._timer.cancel()
            # Allow a few cycles for last TF/msgs to flush
            self.create_timer(0.5, lambda: rclpy.shutdown())
            return

        scan_name = self._scan_files[self._scan_index]
        scan_path = os.path.join(self._velodyne_path, scan_name)

        # Load point cloud - KITTI Velodyne is already (x forward, y left, z up)
        # which matches ROS REP-103, so NO transformation needed!
        pts = self._read_velodyne_bin(scan_path)  # (N,4) float32

        now = self.get_clock().now().to_msg()

        # Publish cloud directly - no coordinate transformation
        pc_msg = make_pointcloud2(pts, self._lidar_frame, now)
        self._pc_pub.publish(pc_msg)

        # Publish map->base_link TF and PoseStamped if poses available
        if self._use_cam0_poses and self._scan_index < self._poses.shape[0]:
            # Camera pose in KITTI camera coordinates
            T_map_cam0_kitti = self._poses[self._scan_index]
            # Convert camera pose to ROS base_link coordinates
            T_map_base_ros = R4_CAM @ T_map_cam0_kitti @ R4_CAM_inv
            x, y, z, qx, qy, qz, qw = T_to_xyzw(T_map_base_ros)

            # PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = self._map_frame
            pose.header.stamp = now
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            self._pose_pub.publish(pose)
            
            # TF
            tfs = TransformStamped()
            tfs.header.stamp = now
            tfs.header.frame_id = self._map_frame
            tfs.child_frame_id = self._base_frame
            tfs.transform.translation.x = float(x)
            tfs.transform.translation.y = float(y)
            tfs.transform.translation.z = float(z)
            tfs.transform.rotation.x = float(qx)
            tfs.transform.rotation.y = float(qy)
            tfs.transform.rotation.z = float(qz)
            tfs.transform.rotation.w = float(qw)
            self._tf_broadcaster.sendTransform(tfs)

        self._scan_index += 1

    # ---------------------------- Data readers ----------------------------

    def _read_velodyne_bin(self, path: str) -> np.ndarray:
        """
        Read a KITTI Velodyne .bin scan into an (N,4) float32 array [x,y,z,intensity].
        
        KITTI Velodyne coordinates are (x forward, y left, z up) which ALREADY matches
        ROS REP-103, so points are returned as-is without any coordinate transformation.
        """
        pts = np.fromfile(path, dtype=np.float32)
        if pts.size % 4 != 0:
            # Log warning and truncate to multiple of 4 (defensive)
            n4 = (pts.size // 4) * 4
            self.get_logger().warn(
                f"Velodyne file {os.path.basename(path)} size not multiple of 4 floats "
                f"({pts.size} floats). Truncating to {n4}."
            )
            pts = pts[:n4]
        return pts.reshape(-1, 4)

# ---------------------------- main ----------------------------

def main(argv=None):
    """Entrypoint to run the node standalone."""
    rclpy.init(args=argv)
    node = KittiPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()