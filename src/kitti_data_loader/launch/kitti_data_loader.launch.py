"""
KITTI Data Loader Launch File
==============================

This launch file starts the KITTI publisher node with parameters loaded from
the configuration file.

The launch file performs the following:

1. Locates the package share directory
2. Loads parameters from ``config/params.yaml``
3. Launches the ``kitti_publisher`` node

Usage
-----

Basic launch::

    ros2 launch kitti_data_loader kitti_data_loader.launch.py

The node will use default parameters from the configuration file.

Nodes Launched
--------------

kitti_publisher_node
    Publishes KITTI LiDAR point clouds and ground-truth poses.
    
    - **Package**: kitti_data_loader
    - **Executable**: kitti_publisher
    - **Output**: screen
    - **Parameters**: Loaded from config/params.yaml

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
        with parameters from the package's config/params.yaml file.
        
    Notes
    -----
    The function automatically locates the package installation directory
    and constructs the path to the configuration file. This ensures the
    launch file works regardless of where the package is installed.
    
    Examples
    --------
    This function is not typically called directly. Instead, use the
    ROS 2 launch command::
    
        ros2 launch kitti_data_loader kitti_data_loader.launch.py
    """
    pkg_dir = get_package_share_directory('kitti_data_loader')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='kitti_data_loader',
            executable='kitti_publisher',
            name='kitti_publisher_node',
            output='screen',
            parameters=[config_file]
        )
    ])