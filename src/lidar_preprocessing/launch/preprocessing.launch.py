"""
Launch file for the LiDAR preprocessing pipeline.

This launch file starts the `VoxelDownsampler` node and loads its
configuration from a YAML file. It is the primary entry point for running the
preprocessing steps on raw point cloud data.

Nodes Launched:
  - `voxel_downsampler`: Subscribes to raw point clouds, performs voxel-grid
    downsampling, and publishes the result.

Parameters Loaded:
  - `preprocessing_params.yaml`: Contains settings for voxel size, topic names,
    ground filtering, etc. See the configuration guide for details.
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate the launch description for the preprocessing pipeline.

    - Finds the `lidar_preprocessing` package share directory.
    - Constructs the full path to the `preprocessing_params.yaml` file.
    - Creates a `Node` action for the `voxel_downsampler` executable, passing
      the configuration file to it.

    Returns:
        LaunchDescription: The complete launch description object.
    """
    config = PathJoinSubstitution(
        [
            FindPackageShare("lidar_preprocessing"),
            "config",
            "params.yaml",
        ]
    )

    downsampler_node = Node(
        package="lidar_preprocessing",
        executable="voxel_downsampler",
        name="voxel_downsampler",
        parameters=[config],
        output="screen",
    )

    return LaunchDescription([downsampler_node])
