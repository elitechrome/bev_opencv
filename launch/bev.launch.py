from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = FindPackageShare('bev_opencv')
    
    default_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'kitti_config.yaml'
    ])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the config file'
    )

    use_camera_info_arg = DeclareLaunchArgument(
        'use_camera_info',
        default_value='true',
        description='Use CameraInfo topic for intrinsics (true) or config file (false)'
    )

    strategy_arg = DeclareLaunchArgument(
        'strategy',
        default_value='geometric',
        description='Processing strategy: sequential, lut, or geometric'
    )
    
    bev_node = Node(
        package='bev_opencv',
        executable='bev_node',
        name='bev_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'use_camera_info': LaunchConfiguration('use_camera_info'),
            'strategy': LaunchConfiguration('strategy')
        }]
    )

    return LaunchDescription([
        config_file_arg,
        use_camera_info_arg,
        strategy_arg,
        bev_node
    ])
