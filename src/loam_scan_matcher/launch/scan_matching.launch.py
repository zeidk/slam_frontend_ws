#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the LOAM-style scan matcher (NumPy/SciPy ICP).

This launch file starts the scan matcher node which consumes edge/planar
feature topics and publishes odometry (``/odom``), TF (``odom -> base_link``),
and a path (``/trajectory``) suitable for Foxglove Studio visualization.

Overridable Parameters
----------------------
edge_topic : str
    Input edge features topic. Default: "features/edge_cloud".
planar_topic : str
    Input planar features topic. Default: "features/planar_cloud".
voxel_size : float
    Voxel size for downsampling (m). Default: 0.2.
max_icp_iter : int
    Maximum ICP iterations per frame. Default: 20.
icp_dist_thresh : float
    Max correspondence distance (m). Default: 1.0.
knn_normals : int
    k-NN for normal estimation on reference. Default: 20.
frame_odom : str
    World frame id (Default: "odom").
frame_base : str
    Base frame id (Default: "base_link").

Example
-------
ros2 launch loam_scan_matcher scan_matching.launch.py \
    edge_topic:=features/edge_cloud \
    planar_topic:=features/planar_cloud \
    voxel_size:=0.2
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the ROS 2 launch description for the scan matcher node."""
    node = Node(
        package='loam_scan_matcher',
        executable='scan_matcher_node',
        name='loam_scan_matcher',
        output='screen',
        parameters=[{
            'edge_topic': 'features/edge_cloud',
            'planar_topic': 'features/planar_cloud',
            'voxel_size': 0.2,
            'max_icp_iter': 20,
            'icp_dist_thresh': 1.0,
            'knn_normals': 20,
            'frame_odom': 'odom',
            'frame_base': 'base_link',
        }]
    )
    return LaunchDescription([node])
