"""Launch file for LOAM feature extraction.

This module defines the launch description for the LOAM (Lidar Odometry
and Mapping) feature extraction node. It loads the configuration parameters
from a YAML file and launches the feature extraction node using ROS 2's
`launch` system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the ROS 2 launch description for LOAM feature extraction.

    This function constructs and returns a LaunchDescription object that
    defines the configuration and execution parameters for the
    `feature_extractor` node within the `loam_feature_extraction` package.

    The node is configured using the parameter file located at:
    `<package_share>/config/feature_params.yaml`.

    Returns
    -------
    launch.LaunchDescription
        A LaunchDescription object containing the feature extraction node
        and its associated configuration.
    """
    config = PathJoinSubstitution([
        FindPackageShare('loam_feature_extraction'),
        'config',
        'feature_params.yaml'
    ])

    extractor_node = Node(
        package='loam_feature_extraction',
        executable='feature_extractor',
        name='loam_feature_extractor',
        parameters=[config],
        output='screen'
    )

    return LaunchDescription([extractor_node])
