"""
KITTI Data Loader Launch File
==============================

This launch file starts the KITTI publisher node with parameters loaded from
the configuration file. It also allows all parameters to be overridden from
the command line.

The launch file performs the following:

1. Locates the package share directory
2. Loads default parameters from ``config/params.yaml``
3. Declares launch arguments for each parameter, allowing overrides.
4. Launches the ``kitti_publisher`` node with the final parameters.

Usage
-----

Launch with defaults from ``params.yaml``::

    ros2 launch kitti_data_loader kitti_data_loader.launch.py

Launch with overridden parameters::

    ros2 launch kitti_data_loader kitti_data_loader.launch.py \\
        dataset_sequence:=01 \\
        publish_rate_hz:=5.0 \\
        base_frame_id:=map

Nodes Launched
--------------

kitti_publisher_node
    Publishes KITTI LiDAR point clouds and ground-truth poses.
    
    - **Package**: kitti_data_loader
    - **Executable**: kitti_publisher
    - **Output**: screen
    - **Parameters**: Loaded from config/params.yaml and overridden by
      command-line launch arguments.

See Also
--------
:class:`kitti_data_loader.kitti_publisher.KittiPublisher`
    The node class that is launched.
:doc:`/guide/configuration`
    Parameter configuration reference.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for the KITTI data loader.

    This function is called by the ROS 2 launch system to construct the
    launch description containing all nodes and their configurations.

    Returns
    -------
    LaunchDescription
        A launch description containing the kitti_publisher_node configured
        with parameters from the package's config/params.yaml file, with
        overrides available via launch arguments.
    """
    pkg_dir = get_package_share_directory("kitti_data_loader")
    config_file = os.path.join(pkg_dir, "config", "params.yaml")

    # --- Start of Modifications ---

    # 1. Declare all launch arguments, using values from params.yaml as defaults

    # Note: Defaults are hardcoded here to match your params.yaml.
    # A more advanced method could parse the YAML, but this is clearer.
    declare_kitti_data_dir_arg = DeclareLaunchArgument(
        "kitti_data_dir",
        default_value="/home/zeid/github/slam_frontend_ws/data/kitti",
        description="Root folder of the KITTI dataset",
    )

    declare_dataset_sequence_arg = DeclareLaunchArgument(
        "dataset_sequence",
        default_value="s00",  # Was '00'
        description='KITTI sequence identifier (e.g., "s00", "s01")',
    )

    declare_publish_rate_hz_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="10.0",
        description="Publishing frequency in Hz",
    )

    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="/kitti/pointcloud_raw",
        description="Topic for raw point clouds",
    )

    declare_ground_truth_pose_topic_arg = DeclareLaunchArgument(
        "ground_truth_pose_topic",
        default_value="/kitti/ground_truth_pose",
        description="Topic for ground-truth poses",
    )

    declare_base_frame_id_arg = DeclareLaunchArgument(
        "base_frame_id",
        default_value="odom",
        description="Parent frame (e.g., odom or map)",
    )

    declare_lidar_frame_id_arg = DeclareLaunchArgument(
        "lidar_frame_id",
        default_value="velodyne",
        description="LiDAR sensor frame (e.g., velodyne)",
    )

    # 2. Define the Node
    kitti_publisher_node = Node(
        package="kitti_data_loader",
        executable="kitti_publisher",
        name="kitti_publisher_node",
        output="screen",
        # The 'parameters' list loads defaults from the YAML file first,
        # then applies overrides from the launch arguments.
        parameters=[
            config_file,  # Load all defaults from YAML
            {
                # Apply overrides from launch arguments
                "kitti_data_dir": LaunchConfiguration("kitti_data_dir"),
                "dataset_sequence": LaunchConfiguration("dataset_sequence"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "ground_truth_pose_topic": LaunchConfiguration(
                    "ground_truth_pose_topic"
                ),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
            },
        ],
    )

    # 3. Return the LaunchDescription
    return LaunchDescription(
        [
            # Add all declared arguments to the launch description
            declare_kitti_data_dir_arg,
            declare_dataset_sequence_arg,
            declare_publish_rate_hz_arg,
            declare_pointcloud_topic_arg,
            declare_ground_truth_pose_topic_arg,
            declare_base_frame_id_arg,
            declare_lidar_frame_id_arg,
            # Add the node itself
            kitti_publisher_node,
        ]
    )


# --- End of Modifications ---
