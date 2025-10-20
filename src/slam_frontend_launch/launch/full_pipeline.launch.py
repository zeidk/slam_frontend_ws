"""Master launch file for complete SLAM frontend pipeline.

This launch file orchestrates the entire pipeline:
1. KITTI data loader
2. Voxel downsampling
3. LOAM feature extraction

Launches all nodes with appropriate parameters and dependencies.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the complete launch description for SLAM frontend demo."""
    
    # Declare launch arguments
    sequence_arg = DeclareLaunchArgument(
        'sequence',
        default_value='00',
        description='KITTI sequence number'
    )
    
    data_path_arg = DeclareLaunchArgument(
        'data_path',
        default_value='~/slam_frontend_ws/data/kitti',
        description='Path to KITTI dataset'
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.2',
        description='Voxel size for downsampling (meters)'
    )
    
    # Include KITTI data loader launch
    kitti_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kitti_data_loader'),
                'launch',
                'kitti_publisher.launch.py'
            ])
        ])
    )
    
    # Include preprocessing launch
    preprocessing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_preprocessing'),
                'launch',
                'preprocessing.launch.py'
            ])
        ])
    )
    
    # Include feature extraction launch
    feature_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('loam_feature_extraction'),
                'launch',
                'feature_extraction.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        sequence_arg,
        data_path_arg,
        voxel_size_arg,
        kitti_launch,
        preprocessing_launch,
        feature_launch,
    ])