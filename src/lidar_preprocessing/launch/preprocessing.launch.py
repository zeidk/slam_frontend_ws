"""Launch file for LiDAR preprocessing."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for preprocessing."""
    config = PathJoinSubstitution([
        FindPackageShare('lidar_preprocessing'),
        'config',
        'preprocessing_params.yaml'
    ])
    
    downsampler_node = Node(
        package='lidar_preprocessing',
        executable='voxel_downsampler',
        name='voxel_downsampler',
        parameters=[config],
        output='screen'
    )
    
    return LaunchDescription([downsampler_node])