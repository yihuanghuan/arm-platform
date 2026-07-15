from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    manipulator_share = FindPackageShare('manipulator').find('manipulator')
    gazebo_launch = os.path.join(manipulator_share, 'gazebo_arm.launch.py')
    world_path = os.path.join(manipulator_share, 'worlds', 'd435i_apriltag_test.world')
    apriltag_config = os.path.join(manipulator_share, 'apriltag_36h11_00000.yaml')
    arm_urdf_path = os.path.join(manipulator_share, 'arm.urdf')

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
    velocity_command_source_arg = DeclareLaunchArgument(
        'velocity_command_source',
        default_value='student_bridge',
        description='Velocity command source in physical_dynamics mode: student_bridge or external'
    )
    fix_base_to_world_arg = DeclareLaunchArgument(
        'fix_base_to_world',
        default_value='true',
        description='Fix base_link to world in the included Gazebo arm launch'
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
    run_visual_ee_estimator_arg = DeclareLaunchArgument(
        'run_visual_ee_estimator',
        default_value='true',
        description='Run the visual end-effector pose estimator'
    )
    visual_world_frame_arg = DeclareLaunchArgument(
        'visual_world_frame',
        default_value='world',
        description='World frame used by /visual_ee_pose'
    )
    visual_base_frame_arg = DeclareLaunchArgument(
        'visual_base_frame',
        default_value='base_link',
        description='Base frame used as orientation fallback by the visual estimator'
    )
    visual_ee_frame_arg = DeclareLaunchArgument(
        'visual_ee_frame',
        default_value='link6',
        description='End-effector frame estimated from AprilTag detections'
    )
    visual_camera_frame_arg = DeclareLaunchArgument(
        'visual_camera_frame',
        default_value='camera_color_optical_frame',
        description='Camera optical frame used by AprilTag detection TF'
    )
    visual_detected_tag_frame_arg = DeclareLaunchArgument(
        'visual_detected_tag_frame',
        default_value='apriltag_36h11_00000',
        description='AprilTag TF frame published by apriltag_ros'
    )
    visual_world_to_tag_xyz_arg = DeclareLaunchArgument(
        'visual_world_to_tag_xyz',
        default_value='1.23042456 0.000976374 0.35065986',
        description='Fixed world-to-apriltag_ros tag-frame translation'
    )
    visual_world_to_tag_rpy_arg = DeclareLaunchArgument(
        'visual_world_to_tag_rpy',
        default_value='-3.12204785 -1.56214388 3.12103003',
        description='Fixed world-to-apriltag_ros tag-frame RPY'
    )
    visual_detection_timeout_sec_arg = DeclareLaunchArgument(
        'visual_detection_timeout_sec',
        default_value='0.5',
        description='Maximum age of visual detections before /visual_ee_pose_valid is false'
    )
    visual_tf_timeout_sec_arg = DeclareLaunchArgument(
        'visual_tf_timeout_sec',
        default_value='0.1',
        description='TF lookup timeout for the visual end-effector estimator'
    )
    visual_position_estimation_mode_arg = DeclareLaunchArgument(
        'visual_position_estimation_mode',
        default_value='kinematic_orientation',
        description='visual position mode: kinematic_orientation or full_pose'
    )
    run_visual_stabilization_controller_arg = DeclareLaunchArgument(
        'run_visual_stabilization_controller',
        default_value='false',
        description='Run the visual end-effector stabilization controller'
    )
    visual_stabilization_control_rate_arg = DeclareLaunchArgument(
        'visual_stabilization_control_rate',
        default_value='100.0',
        description='Visual stabilization control rate in Hz'
    )
    visual_stabilization_dry_run_arg = DeclareLaunchArgument(
        'visual_stabilization_dry_run',
        default_value='true',
        description='Keep visual stabilization in debug-only mode when true'
    )
    visual_stabilization_control_mode_arg = DeclareLaunchArgument(
        'visual_stabilization_control_mode',
        default_value='xyz',
        description='Visual stabilization mode: xyz or se3_debug'
    )
    visual_stabilization_damping_arg = DeclareLaunchArgument(
        'visual_stabilization_damping',
        default_value='0.05',
        description='Damped pseudo-inverse lambda for visual stabilization CLIK'
    )
    visual_stabilization_max_joint_velocity_arg = DeclareLaunchArgument(
        'visual_stabilization_max_joint_velocity',
        default_value='0.35',
        description='Joint velocity clamp in rad/s'
    )
    visual_stabilization_max_task_velocity_xyz_arg = DeclareLaunchArgument(
        'visual_stabilization_max_task_velocity_xyz',
        default_value='0.08 0.08 0.08',
        description='XYZ task-space velocity clamp in m/s'
    )
    visual_stabilization_position_deadband_arg = DeclareLaunchArgument(
        'visual_stabilization_position_deadband_m',
        default_value='0.003',
        description='XYZ visual position error deadband in meters'
    )
    visual_stabilization_joint_limit_margin_arg = DeclareLaunchArgument(
        'visual_stabilization_joint_limit_margin_rad',
        default_value='0.05',
        description='Stop if a command would push a joint farther into this limit margin'
    )
    visual_stabilization_measurement_timeout_arg = DeclareLaunchArgument(
        'visual_stabilization_measurement_timeout',
        default_value='0.35',
        description='Maximum visual measurement age before zero velocity'
    )
    visual_stabilization_joint_state_timeout_arg = DeclareLaunchArgument(
        'visual_stabilization_joint_state_timeout',
        default_value='0.35',
        description='Maximum joint state age before zero velocity'
    )
    visual_stabilization_command_topic_arg = DeclareLaunchArgument(
        'visual_stabilization_command_topic',
        default_value='/arm_velocity_controller/commands',
        description='Velocity controller command topic used when dry_run is false'
    )
    visual_stabilization_command_start_delay_arg = DeclareLaunchArgument(
        'visual_stabilization_command_start_delay_sec',
        default_value='0.0',
        description='Delay command publishing after target lock; useful for phase 3 pulse tests'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'world': world_path,
            'camera_mount_mode': LaunchConfiguration('camera_mount_mode'),
            'control_mode': LaunchConfiguration('control_mode'),
            'velocity_command_source': LaunchConfiguration('velocity_command_source'),
            'camera_enabled': 'true',
            'rgbd_enabled': 'true',
            'rgbd_frame_name': 'camera_color_optical_frame',
            'imu_enabled': 'true',
            'use_ros2_control': 'true',
            'fix_base_to_world': LaunchConfiguration('fix_base_to_world'),
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

    visual_ee_estimator = Node(
        condition=IfCondition(LaunchConfiguration('run_visual_ee_estimator')),
        package='manipulator',
        executable='visual_ee_pose_estimator.py',
        name='visual_ee_pose_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world_frame': LaunchConfiguration('visual_world_frame'),
            'base_frame': LaunchConfiguration('visual_base_frame'),
            'ee_frame': LaunchConfiguration('visual_ee_frame'),
            'camera_frame': LaunchConfiguration('visual_camera_frame'),
            'detected_tag_frame': LaunchConfiguration('visual_detected_tag_frame'),
            'world_to_tag_xyz': LaunchConfiguration('visual_world_to_tag_xyz'),
            'world_to_tag_rpy': LaunchConfiguration('visual_world_to_tag_rpy'),
            'position_estimation_mode': LaunchConfiguration('visual_position_estimation_mode'),
            'detection_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_detection_timeout_sec'),
                value_type=float),
            'tf_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_tf_timeout_sec'),
                value_type=float),
        }]
    )

    visual_stabilization_controller = Node(
        condition=IfCondition(LaunchConfiguration('run_visual_stabilization_controller')),
        package='manipulator',
        executable='visual_ee_stabilization_controller.py',
        name='visual_ee_stabilization_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'urdf_path': arm_urdf_path,
            'ee_frame': LaunchConfiguration('visual_ee_frame'),
            'dry_run': ParameterValue(
                LaunchConfiguration('visual_stabilization_dry_run'),
                value_type=bool),
            'control_mode': LaunchConfiguration('visual_stabilization_control_mode'),
            'command_topic': LaunchConfiguration('visual_stabilization_command_topic'),
            'command_start_delay_sec': ParameterValue(
                LaunchConfiguration('visual_stabilization_command_start_delay_sec'),
                value_type=float),
            'control_rate': ParameterValue(
                LaunchConfiguration('visual_stabilization_control_rate'),
                value_type=float),
            'damping': ParameterValue(
                LaunchConfiguration('visual_stabilization_damping'),
                value_type=float),
            'max_joint_velocity': ParameterValue(
                LaunchConfiguration('visual_stabilization_max_joint_velocity'),
                value_type=float),
            'max_task_velocity_xyz': LaunchConfiguration(
                'visual_stabilization_max_task_velocity_xyz'),
            'position_deadband_m': ParameterValue(
                LaunchConfiguration('visual_stabilization_position_deadband_m'),
                value_type=float),
            'joint_limit_margin_rad': ParameterValue(
                LaunchConfiguration('visual_stabilization_joint_limit_margin_rad'),
                value_type=float),
            'measurement_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_stabilization_measurement_timeout'),
                value_type=float),
            'joint_state_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_stabilization_joint_state_timeout'),
                value_type=float),
        }]
    )

    return LaunchDescription([
        gui_arg,
        use_rviz_arg,
        camera_mount_mode_arg,
        control_mode_arg,
        velocity_command_source_arg,
        fix_base_to_world_arg,
        use_sim_time_arg,
        run_validator_arg,
        validator_duration_arg,
        validator_output_csv_arg,
        run_dynamic_logger_arg,
        dynamic_logger_duration_arg,
        dynamic_logger_output_csv_arg,
        run_mount_regression_arg,
        mount_regression_output_csv_arg,
        run_visual_ee_estimator_arg,
        visual_world_frame_arg,
        visual_base_frame_arg,
        visual_ee_frame_arg,
        visual_camera_frame_arg,
        visual_detected_tag_frame_arg,
        visual_world_to_tag_xyz_arg,
        visual_world_to_tag_rpy_arg,
        visual_detection_timeout_sec_arg,
        visual_tf_timeout_sec_arg,
        visual_position_estimation_mode_arg,
        run_visual_stabilization_controller_arg,
        visual_stabilization_control_rate_arg,
        visual_stabilization_dry_run_arg,
        visual_stabilization_control_mode_arg,
        visual_stabilization_damping_arg,
        visual_stabilization_max_joint_velocity_arg,
        visual_stabilization_max_task_velocity_xyz_arg,
        visual_stabilization_position_deadband_arg,
        visual_stabilization_joint_limit_margin_arg,
        visual_stabilization_measurement_timeout_arg,
        visual_stabilization_joint_state_timeout_arg,
        visual_stabilization_command_topic_arg,
        visual_stabilization_command_start_delay_arg,
        gazebo,
        apriltag,
        visual_ee_estimator,
        visual_stabilization_controller,
        validator,
        dynamic_logger,
        mount_regression,
    ])
