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
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo /clock for AprilTag test nodes'
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
    run_dynamic_logger_arg = DeclareLaunchArgument(
        'run_dynamic_logger',
        default_value='false',
        description='Run the timestamp-synchronized dynamic ground-truth logger'
    )
    dynamic_logger_duration_arg = DeclareLaunchArgument(
        'dynamic_logger_duration',
        default_value='60.0',
        description='Dynamic ground-truth logger duration in seconds'
    )
    dynamic_logger_output_csv_arg = DeclareLaunchArgument(
        'dynamic_logger_output_csv',
        default_value='/tmp/d435i_dynamic_ground_truth.csv',
        description='CSV path for the dynamic ground-truth logger'
    )
    run_mount_regression_arg = DeclareLaunchArgument(
        'run_mount_regression',
        default_value='false',
        description='Run the base camera mount regression helper'
    )
    mount_regression_output_csv_arg = DeclareLaunchArgument(
        'mount_regression_output_csv',
        default_value='/tmp/d435i_base_camera_regression.csv',
        description='CSV path for base camera mount regression'
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        output='screen',
        parameters=[apriltag_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '--duration', LaunchConfiguration('validator_duration'),
            '--output-csv', LaunchConfiguration('validator_output_csv'),
        ]
    )

    dynamic_logger = Node(
        condition=IfCondition(LaunchConfiguration('run_dynamic_logger')),
        package='manipulator',
        executable='dynamic_ground_truth_logger.py',
        name='dynamic_ground_truth_logger',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '--duration', LaunchConfiguration('dynamic_logger_duration'),
            '--output-csv', LaunchConfiguration('dynamic_logger_output_csv'),
        ]
    )

    mount_regression = Node(
        condition=IfCondition(LaunchConfiguration('run_mount_regression')),
        package='manipulator',
        executable='check_camera_mount_regression.py',
        name='check_camera_mount_regression',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '--output-csv', LaunchConfiguration('mount_regression_output_csv'),
        ]
    )

    return LaunchDescription([
        gui_arg,
        use_rviz_arg,
        camera_mount_mode_arg,
        control_mode_arg,
        use_sim_time_arg,
        run_validator_arg,
        validator_duration_arg,
        validator_output_csv_arg,
        run_dynamic_logger_arg,
        dynamic_logger_duration_arg,
        dynamic_logger_output_csv_arg,
        run_mount_regression_arg,
        mount_regression_output_csv_arg,
        gazebo,
        apriltag,
        validator,
        dynamic_logger,
        mount_regression,
    ])
