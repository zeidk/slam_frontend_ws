"""
Launch File: feature_extraction.launch.py
=========================================

Description
-----------
This launch file orchestrates the full LOAM (LiDAR Odometry and Mapping)
feature extraction pipeline. It sequentially launches three key components
to form a modular, simulation-ready LiDAR processing chain:

1. **KITTI Data Loader (`kitti_data_loader.launch.py`)**
   - Publishes raw KITTI LiDAR and ground-truth pose data as ROS 2 topics.
   - Emulates sensor playback from the KITTI odometry dataset.

2. **LiDAR Preprocessing (`preprocessing.launch.py`)**
   - Applies voxel-grid downsampling, optional ground filtering,
     and other configurable preprocessing steps.
   - Subscribes to `/kitti/pointcloud_raw` and publishes a reduced-resolution
     cloud on `/preprocessing/downsampled_cloud`.

3. **Feature Extraction (`feature_extractor`)**
   - Consumes the downsampled point cloud and extracts geometric features
     (e.g., edge and planar points) for subsequent odometry and mapping stages.

This configuration is intended to be used with the NIST SLAM front-end
workspace, but it can be adapted for other LiDAR datasets and sensors by
changing the parameter files.

Execution
---------
To launch the entire pipeline:

.. code-block:: bash

   ros2 launch loam_feature_extraction feature_extraction.launch.py

Upon execution, the following nodes are launched:

- **`kitti_publisher`** — Publishes KITTI LiDAR scans and ground-truth poses.
- **`voxel_downsampler`** — Performs voxel-grid downsampling on incoming point clouds.
- **`loam_feature_extractor`** — Extracts edge and planar features from preprocessed clouds.

Expected Topics
---------------
+--------------------------------------+--------------------------------------------+
| **Input**                            | **Output**                                 |
+--------------------------------------+--------------------------------------------+
| `/kitti/pointcloud_raw`              | `/preprocessing/downsampled_cloud`         |
| `/preprocessing/downsampled_cloud`   | `/features/edge_cloud`, `/features/planar_cloud` |
+--------------------------------------+--------------------------------------------+

Configuration Files
-------------------
- ``lidar_preprocessing/config/params.yaml`` — Parameters for preprocessing
  (voxel size, filtering thresholds, topic names).
- ``loam_feature_extraction/config/params.yaml`` — Parameters for
  feature extraction (ring count, curvature thresholds, etc.).
- ``kitti_data_loader/config/params.yaml`` — Dataset paths and topic mappings.

Returns
-------
launch.LaunchDescription
    A ROS 2 launch description object containing all nodes and their
    configurations for the LOAM feature extraction pipeline.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the ROS 2 launch description for the complete LOAM feature
    extraction pipeline.

    This function composes and returns a `LaunchDescription` object that
    performs the following:
      1. Includes the `preprocessing.launch.py` file from the
         `lidar_preprocessing` package. This, in turn, launches both
         the `kitti_publisher` and `voxel_downsampler` nodes.
      2. Launches the `loam_feature_extractor` node configured with
         parameters defined in `params.yaml`.

    The resulting pipeline provides an end-to-end LiDAR data processing
    workflow — from dataset playback to feature extraction — suitable for
    real-time visualization in Foxglove Studio or RViz.

    Returns
    -------
    launch.LaunchDescription
        The composed launch description containing all dependent nodes.
    """
    # -------------------------------------------------------
    # Include LiDAR preprocessing (which also includes KITTI loader)
    # -------------------------------------------------------
    preprocessing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lidar_preprocessing"),
                "launch",
                "preprocessing.launch.py",
            ])
        )
    )

    # -------------------------------------------------------
    # Feature extraction configuration and node
    # -------------------------------------------------------
    feature_config = PathJoinSubstitution([
        FindPackageShare("loam_feature_extraction"),
        "config",
        "params.yaml",
    ])

    feature_extractor_node = Node(
        package="loam_feature_extraction",
        executable="feature_extractor",
        name="loam_feature_extractor",
        parameters=[feature_config],
        output="screen",
    )

    # -------------------------------------------------------
    # Launch both preprocessing (with KITTI loader) and feature extractor
    # -------------------------------------------------------
    return LaunchDescription([
        preprocessing_launch,
        feature_extractor_node,
    ])
