#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch File: scan_matching.launch.py
====================================

Description
-----------
This launch file starts the **LOAM scan matching** stage and, by inclusion,
its upstream dependencies. It first launches the **feature extraction pipeline**
(which itself brings up KITTI playback and LiDAR preprocessing), then launches
the scan matcher node configured via a package-local YAML file.

Pipeline (launch order)
-----------------------
1) **Feature Extraction Pipeline** (included from
   ``loam_feature_extraction/launch/feature_extraction.launch.py``)
   - KITTI Data Loader (raw LiDAR + ground truth)
   - LiDAR Preprocessing (e.g., voxel downsampling)
   - LOAM Feature Extraction (publishes edge/planar feature clouds)

2) **LOAM Scan Matcher** (this file)
   - Subscribes to: ``features/edge_cloud`` and ``features/planar_cloud``
   - Publishes: odometry (``/odom``), TF (``odom -> base_link``), and path (``/trajectory``)

Configuration
-------------
All scan matcher parameters are loaded from:
``loam_scan_matcher/config/params.yaml``

**No inline parameter overrides** are provided in this launch file, ensuring that
edits to the YAML fully control behavior (topics, frames, ICP settings, etc.).

Execution
---------
Launch the full chain (KITTI → Preprocessing → Features → Scan Matching):

.. code-block:: bash

   ros2 launch loam_scan_matcher scan_matching.launch.py

Returns
-------
launch.LaunchDescription
    A ROS 2 launch description that includes the feature extraction pipeline
    and the LOAM scan matcher node, with parameters sourced from YAML only.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Compose the launch description for LOAM scan matching.

    Steps:
      1) Include the **feature extraction** launch file from the
         ``loam_feature_extraction`` package so that KITTI playback and LiDAR
         preprocessing are available before scan matching starts.
      2) Launch the **scan matcher** node from the ``loam_scan_matcher`` package,
         loading its configuration solely from ``config/params.yaml``.

    Returns:
        launch.LaunchDescription: The composed launch description.
    """
    # ------------------------------------------------------------------
    # Include LOAM Feature Extraction Pipeline
    # (KITTI loader + preprocessing + feature extractor)
    # ------------------------------------------------------------------
    feature_extraction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("loam_feature_extraction"),
                "launch",
                "feature_extraction.launch.py",
            ])
        )
    )

    # ------------------------------------------------------------------
    # Scan Matcher parameters from YAML (no inline overrides)
    # ------------------------------------------------------------------
    scan_matcher_params = PathJoinSubstitution([
        FindPackageShare("loam_scan_matcher"),
        "config",
        "params.yaml",
    ])

    # ------------------------------------------------------------------
    # LOAM Scan Matcher Node
    # ------------------------------------------------------------------
    scan_matcher_node = Node(
        package="loam_scan_matcher",
        executable="loam_scan_matcher",
        name="loam_scan_matcher",
        output="screen",
        parameters=[scan_matcher_params],  # <-- YAML-driven configuration only
    )

    # ------------------------------------------------------------------
    # Launch: feature extraction first, then scan matcher
    # ------------------------------------------------------------------
    return LaunchDescription([
        feature_extraction_launch,
        scan_matcher_node,
    ])
