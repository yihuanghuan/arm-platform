from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    livox_driver_path = get_package_share_directory('livox_ros_driver2')
    
    xfer_format_arg = DeclareLaunchArgument(
        'xfer_format',
        default_value='1', # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
        description='xfer_format: 0-Pointcloud2, 1-customized pointcloud'
    )
    
    multi_topic_arg = DeclareLaunchArgument(
        'multi_topic',
        default_value='1',# 0-All LiDARs share the same topic, 1-One LiDAR one topic
        description='multi_topic: 0-All LiDARs share same topic, 1-One LiDAR one topic'
    )
    
    left_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            livox_driver_path + '/launch_ROS2/msg_MID360_launch.py'
        ),
        launch_arguments={
            'lidar_config': 'left',
            'xfer_format': LaunchConfiguration('xfer_format'),
            'multi_topic': LaunchConfiguration('multi_topic')
        }.items()
    )
    
    right_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            livox_driver_path + '/launch_ROS2/msg_MID360_launch.py'
        ),
        launch_arguments={
            'lidar_config': 'right',
            'xfer_format': LaunchConfiguration('xfer_format'),
            'multi_topic': LaunchConfiguration('multi_topic')
        }.items()
    )


    return LaunchDescription([
        xfer_format_arg,
        multi_topic_arg,
        # left_lidar_launch,
        right_lidar_launch
    ])
