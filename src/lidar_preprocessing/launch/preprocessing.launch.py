"""
Launch file for the LiDAR preprocessing pipeline and KITTI data loader.

This launch file starts:
  - `kitti_publisher`: Publishes raw KITTI LiDAR and pose data.
  - `voxel_downsampler`: Subscribes to raw point clouds, performs voxel-grid
    downsampling, and publishes the result.

Parameters Loaded:
  - `preprocessing_params.yaml`: Settings for voxel size, topics, and filtering.
  - `kitti_data_loader/config/params.yaml`: KITTI dataset and topic settings.
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate the launch description for preprocessing and KITTI data loading.

    - Launches the KITTI publisher node (from `kitti_data_loader.launch.py`)
    - Launches the voxel downsampler node (from `lidar_preprocessing`)
    """
    # Paths to configuration files
    preprocessing_config = PathJoinSubstitution(
        [FindPackageShare("lidar_preprocessing"), "config", "params.yaml"]
    )

    # Include the KITTI data loader launch file
    kitti_loader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kitti_data_loader"), "launch", "kitti_data_loader.launch.py"]
            )
        )
    )

    # Define the voxel downsampler node
    downsampler_node = Node(
        package="lidar_preprocessing",
        executable="voxel_downsampler",
        name="voxel_downsampler",
        parameters=[preprocessing_config],
        output="screen",
    )

    # Launch both the KITTI loader and the preprocessing node
    return LaunchDescription([kitti_loader_launch, downsampler_node])
