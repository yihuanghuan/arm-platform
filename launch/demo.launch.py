from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    manipulator_pkg = FindPackageShare('manipulator')
    
    loc_type_arg = DeclareLaunchArgument(
        'loc_type',
        default_value='mid360',
        description='Choose localization type: vicon or mid360'
    )
    
    use_slave_mode_arg = DeclareLaunchArgument(
        'use_slave_mode',
        default_value='True',
        description='If true, use slave_arm + camera'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([manipulator_pkg, 'lidar.launch.py'])
        ),
    )
    
    slave_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([manipulator_pkg, 'slave_arm.launch.py'])
        ),
    )
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([manipulator_pkg, 'camera.launch.py'])
        ),
    )
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([manipulator_pkg, 'localization.launch.py'])
        ),
        launch_arguments={'loc_type': LaunchConfiguration('loc_type')}.items()
    )

    return LaunchDescription([
        use_slave_mode_arg,
        loc_type_arg,
        lidar_launch,
        # slave_arm_launch,
        # camera_launch,
        localization_launch,
    ])
