from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    manipulator_share = FindPackageShare('manipulator').find('manipulator')
    gazebo_launch = os.path.join(manipulator_share, 'gazebo_arm.launch.py')
    world_path = os.path.join(manipulator_share, 'worlds', 'd435i_apriltag_test.world')
    apriltag_config = os.path.join(manipulator_share, 'apriltag_36h11_00000.yaml')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Start the Gazebo Classic GUI client'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz from the included Gazebo arm launch'
    )
    camera_mount_mode_arg = DeclareLaunchArgument(
        'camera_mount_mode',
        default_value='ee',
        description='D435i mount preset: ee or base'
    )
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='kinematic_visualization',
        description='Gazebo control mode for the included arm launch'
    )
    run_validator_arg = DeclareLaunchArgument(
        'run_validator',
        default_value='false',
        description='Run the AprilTag ground-truth comparison helper'
    )
    validator_duration_arg = DeclareLaunchArgument(
        'validator_duration',
        default_value='20.0',
        description='Validation duration in seconds when run_validator is true'
    )
    validator_output_csv_arg = DeclareLaunchArgument(
        'validator_output_csv',
        default_value='/tmp/d435i_apriltag_validation.csv',
        description='CSV path for validation samples when run_validator is true'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'world': world_path,
            'camera_mount_mode': LaunchConfiguration('camera_mount_mode'),
            'control_mode': LaunchConfiguration('control_mode'),
            'camera_enabled': 'true',
            'rgbd_enabled': 'true',
            'rgbd_frame_name': 'camera_color_optical_frame',
            'imu_enabled': 'true',
            'use_ros2_control': 'true',
        }.items()
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        output='screen',
        parameters=[apriltag_config],
        remappings=[
            ('image_rect', '/d435i/color/image_raw'),
            ('camera_info', '/d435i/color/camera_info'),
        ]
    )

    validator = Node(
        condition=IfCondition(LaunchConfiguration('run_validator')),
        package='manipulator',
        executable='check_apriltag_ground_truth.py',
        name='check_apriltag_ground_truth',
        output='screen',
        arguments=[
            '--duration', LaunchConfiguration('validator_duration'),
            '--output-csv', LaunchConfiguration('validator_output_csv'),
        ]
    )

    return LaunchDescription([
        gui_arg,
        use_rviz_arg,
        camera_mount_mode_arg,
        control_mode_arg,
        run_validator_arg,
        validator_duration_arg,
        validator_output_csv_arg,
        gazebo,
        apriltag,
        validator,
    ])
