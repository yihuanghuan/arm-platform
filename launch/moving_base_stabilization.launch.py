from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def _validate_arguments(context, *args, **kwargs):
    experiment_mode = LaunchConfiguration('experiment_mode').perform(context)
    if experiment_mode not in (
            'plant_test', 'baseline', 'visual_xyz', 'ground_truth_xyz'):
        raise RuntimeError(
            'experiment_mode must be plant_test, baseline, visual_xyz, '
            'or ground_truth_xyz')
    return []


def generate_launch_description():
    manipulator_share = FindPackageShare('manipulator').find('manipulator')
    gazebo_launch = os.path.join(manipulator_share, 'gazebo_arm.launch.py')
    default_world = os.path.join(
        manipulator_share, 'worlds', 'd435i_apriltag_board_2x2.world')
    default_apriltag_config = os.path.join(
        manipulator_share, 'apriltag_36h11_board_2x2.yaml')
    arm_urdf_path = os.path.join(manipulator_share, 'arm.urdf')
    baseline_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('experiment_mode'), "' == 'baseline'"]))
    visual_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('experiment_mode'), "' == 'visual_xyz'"]))
    ground_truth_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('experiment_mode'), "' == 'ground_truth_xyz'"]))
    diagnostics_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('experiment_mode'), "' == 'visual_xyz' and '",
        LaunchConfiguration('run_phase4_diagnostics'),
        "'.lower() in ('1', 'true', 'yes', 'on')"]))
    transform_chain_diagnostics_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('experiment_mode'), "' == 'visual_xyz' and '",
        LaunchConfiguration('run_phase4_transform_chain_diagnostics'),
        "'.lower() in ('1', 'true', 'yes', 'on')"]))

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Start Gazebo Classic GUI client')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world file')
    apriltag_config_arg = DeclareLaunchArgument(
        'apriltag_config',
        default_value=default_apriltag_config,
        description='apriltag_ros YAML config file')
    camera_mount_mode_arg = DeclareLaunchArgument(
        'camera_mount_mode',
        default_value='ee',
        description='D435i mount preset: ee or base')
    experiment_mode_arg = DeclareLaunchArgument(
        'experiment_mode',
        default_value='baseline',
        description=(
            'Experiment mode: plant_test, baseline, visual_xyz, '
            'or ground_truth_xyz'))
    disturbance_csv_arg = DeclareLaunchArgument(
        'disturbance_csv',
        default_value='/tmp/base_disturbance.csv',
        description='Base disturbance trajectory CSV')
    replay_output_csv_arg = DeclareLaunchArgument(
        'replay_output_csv',
        default_value='/tmp/base_disturbance_replay.csv',
        description='Replay metrics and Gazebo ground-truth CSV')
    camera_entity_name_arg = DeclareLaunchArgument(
        'camera_entity_name',
        default_value='windylab_arm::link6',
        description=(
            'Gazebo entity used for camera GT. Gazebo collapses fixed D435i links, '
            'so EE mode uses the carrier link6 by default.'))
    replay_rate_hz_arg = DeclareLaunchArgument(
        'replay_rate_hz',
        default_value='100.0',
        description='Base replay target rate')
    replay_clock_source_arg = DeclareLaunchArgument(
        'replay_clock_source',
        default_value='sim',
        description='Replay scheduling clock: sim or wall')
    replay_service_call_timeout_arg = DeclareLaunchArgument(
        'replay_service_call_timeout',
        default_value='0.2',
        description='Timeout for each Gazebo set/get entity state service call')
    replay_state_sample_stride_arg = DeclareLaunchArgument(
        'replay_state_sample_stride',
        default_value='1',
        description='Compatibility option; replay now samples Gazebo state from topics')
    replay_gt_max_abs_position_m_arg = DeclareLaunchArgument(
        'replay_gt_max_abs_position_m',
        default_value='5.0',
        description='Reject Gazebo GT samples with any position magnitude above this')
    replay_entity_stable_samples_arg = DeclareLaunchArgument(
        'replay_entity_stable_samples',
        default_value='3',
        description='Required consecutive physical GT samples before replay starts')
    baseline_rate_hz_arg = DeclareLaunchArgument(
        'baseline_rate_hz',
        default_value='100.0',
        description='Baseline zero velocity command rate')
    visual_stabilization_control_rate_arg = DeclareLaunchArgument(
        'visual_stabilization_control_rate',
        default_value='100.0',
        description='Visual XYZ stabilization control rate')
    visual_stabilization_max_joint_velocity_arg = DeclareLaunchArgument(
        'visual_stabilization_max_joint_velocity',
        default_value='0.2',
        description='Visual XYZ joint velocity clamp in rad/s')
    visual_stabilization_max_task_velocity_xyz_arg = DeclareLaunchArgument(
        'visual_stabilization_max_task_velocity_xyz',
        default_value='0.05 0.05 0.05',
        description='Visual XYZ task velocity clamp in m/s')
    visual_stabilization_position_deadband_arg = DeclareLaunchArgument(
        'visual_stabilization_position_deadband_m',
        default_value='0.003',
        description='Visual XYZ position deadband in meters')
    visual_stabilization_measurement_timeout_arg = DeclareLaunchArgument(
        'visual_stabilization_measurement_timeout',
        default_value='0.80',
        description='Maximum visual measurement age before zero velocity')
    visual_stabilization_joint_state_timeout_arg = DeclareLaunchArgument(
        'visual_stabilization_joint_state_timeout',
        default_value='0.35',
        description='Maximum joint-state age before zero velocity')
    visual_required_consecutive_valid_poses_arg = DeclareLaunchArgument(
        'visual_required_consecutive_valid_poses',
        default_value='3',
        description='Valid visual pose samples required before locking a target')
    visual_relock_after_visual_loss_sec_arg = DeclareLaunchArgument(
        'visual_relock_after_visual_loss_sec',
        default_value='0.5',
        description='Reset locked target after this much continuous visual loss')
    visual_max_visual_error_norm_m_arg = DeclareLaunchArgument(
        'visual_max_visual_error_norm_m',
        default_value='0.20',
        description='Safety stop threshold if visual XYZ error exceeds this')
    visual_target_relock_enabled_arg = DeclareLaunchArgument(
        'visual_target_relock_enabled',
        default_value='true',
        description='Allow target relock/reset after sustained visual loss')
    visual_stop_on_large_error_arg = DeclareLaunchArgument(
        'visual_stop_on_large_error',
        default_value='true',
        description='Safety-stop instead of relocking when visual error is too large')
    visual_max_joint_acceleration_arg = DeclareLaunchArgument(
        'visual_max_joint_acceleration_rad_s2',
        default_value='0.3',
        description='Joint command acceleration clamp in rad/s^2')
    ground_truth_control_rate_arg = DeclareLaunchArgument(
        'ground_truth_control_rate',
        default_value='100.0',
        description='Gazebo GT XYZ stabilization control rate')
    ground_truth_max_joint_velocity_arg = DeclareLaunchArgument(
        'ground_truth_max_joint_velocity',
        default_value='1.5',
        description='Gazebo GT XYZ joint velocity clamp in rad/s')
    ground_truth_max_task_velocity_xyz_arg = DeclareLaunchArgument(
        'ground_truth_max_task_velocity_xyz',
        default_value='0.35 0.35 0.35',
        description='Gazebo GT XYZ task velocity clamp in m/s')
    ground_truth_max_joint_acceleration_arg = DeclareLaunchArgument(
        'ground_truth_max_joint_acceleration_rad_s2',
        default_value='3.0',
        description='Gazebo GT joint command acceleration clamp in rad/s^2')
    ground_truth_task_gain_arg = DeclareLaunchArgument(
        'ground_truth_task_gain',
        default_value='5.0 5.0 5.0',
        description='Gazebo GT XYZ proportional task gain')
    ground_truth_damping_arg = DeclareLaunchArgument(
        'ground_truth_damping',
        default_value='0.02',
        description='Gazebo GT damped pseudoinverse coefficient')
    visual_detection_timeout_sec_arg = DeclareLaunchArgument(
        'visual_detection_timeout_sec',
        default_value='0.80',
        description='Maximum AprilTag detection age before visual pose invalid')
    visual_tf_timeout_sec_arg = DeclareLaunchArgument(
        'visual_tf_timeout_sec',
        default_value='0.1',
        description='TF lookup timeout for visual end-effector pose estimation')
    visual_tag_tf_mode_arg = DeclareLaunchArgument(
        'visual_tag_tf_mode',
        default_value='latest',
        description='Tag TF lookup mode for visual estimation: stamped or latest')
    visual_max_tag_tf_age_sec_arg = DeclareLaunchArgument(
        'visual_max_tag_tf_age_sec',
        default_value='1.5',
        description='Maximum latest camera->tag TF age before visual pose invalid')
    visual_position_estimation_mode_arg = DeclareLaunchArgument(
        'visual_position_estimation_mode',
        default_value='kinematic_orientation',
        description='Visual pose mode: kinematic_orientation or full_pose')
    visual_primary_detected_tag_frame_arg = DeclareLaunchArgument(
        'visual_primary_detected_tag_frame',
        default_value='apriltag_36h11_00000',
        description='Primary tag frame used by diagnostics and single-tag fallback')
    visual_tag_ids_arg = DeclareLaunchArgument(
        'visual_tag_ids',
        default_value='0 1 2 3',
        description='Configured AprilTag ids for multi-tag visual estimation')
    visual_detected_tag_frames_arg = DeclareLaunchArgument(
        'visual_detected_tag_frames',
        default_value=(
            'apriltag_36h11_00000 apriltag_36h11_00001 '
            'apriltag_36h11_00002 apriltag_36h11_00003'),
        description='Detected AprilTag TF frames matching visual_tag_ids')
    visual_world_to_tag_xyzs_arg = DeclareLaunchArgument(
        'visual_world_to_tag_xyzs',
        default_value=(
            '1.600424560 0.150976374 0.200659860;'
            '1.600424560 0.150976374 0.500659860;'
            '1.600424560 -0.149023626 0.200659860;'
            '1.600424560 -0.149023626 0.500659860'),
        description='Semicolon-separated world->tag XYZ triples matching visual_tag_ids')
    visual_world_to_tag_rpys_arg = DeclareLaunchArgument(
        'visual_world_to_tag_rpys',
        default_value=(
            '-3.12204785 -1.56214388 3.12103003;'
            '-3.12204785 -1.56214388 3.12103003;'
            '-3.12204785 -1.56214388 3.12103003;'
            '-3.12204785 -1.56214388 3.12103003'),
        description='Semicolon-separated world->tag RPY triples matching visual_tag_ids')
    rgbd_update_rate_arg = DeclareLaunchArgument(
        'rgbd_update_rate',
        default_value='15',
        description='RGB-D sensor update rate passed to gazebo_arm.launch.py')
    rgbd_width_arg = DeclareLaunchArgument(
        'rgbd_width',
        default_value='320',
        description='RGB-D image width passed to gazebo_arm.launch.py')
    rgbd_height_arg = DeclareLaunchArgument(
        'rgbd_height',
        default_value='240',
        description='RGB-D image height passed to gazebo_arm.launch.py')
    imu_enabled_arg = DeclareLaunchArgument(
        'imu_enabled',
        default_value='true',
        description='Attach the Gazebo D435i IMU plugin')
    imu_update_rate_arg = DeclareLaunchArgument(
        'imu_update_rate',
        default_value='200',
        description='Gazebo D435i IMU update rate in Hz')
    run_phase4_diagnostics_arg = DeclareLaunchArgument(
        'run_phase4_diagnostics',
        default_value='false',
        description='Start phase4_visual_chain_diagnostics.py in visual_xyz mode')
    phase4_diagnostics_output_csv_arg = DeclareLaunchArgument(
        'phase4_diagnostics_output_csv',
        default_value='/tmp/phase4_visual_chain_diagnostics.csv',
        description='CSV output path for phase 4 visual chain diagnostics')
    phase4_diagnostics_duration_sec_arg = DeclareLaunchArgument(
        'phase4_diagnostics_duration_sec',
        default_value='0.0',
        description='Diagnostics duration; 0 means run until launch exits')
    phase4_diagnostics_sample_hz_arg = DeclareLaunchArgument(
        'phase4_diagnostics_sample_hz',
        default_value='30.0',
        description='Diagnostics CSV sample rate in Hz')
    run_phase4_transform_chain_diagnostics_arg = DeclareLaunchArgument(
        'run_phase4_transform_chain_diagnostics',
        default_value='false',
        description='Start phase4.3 transform-chain diagnostics in visual_xyz mode')
    phase4_transform_chain_output_csv_arg = DeclareLaunchArgument(
        'phase4_transform_chain_output_csv',
        default_value='/tmp/phase4_3_visual_pose_transform_chain.csv',
        description='CSV output path for phase 4.3 transform-chain diagnostics')
    phase4_transform_chain_duration_sec_arg = DeclareLaunchArgument(
        'phase4_transform_chain_duration_sec',
        default_value='0.0',
        description='Transform-chain diagnostics duration; 0 means run until launch exits')
    phase4_transform_chain_sample_hz_arg = DeclareLaunchArgument(
        'phase4_transform_chain_sample_hz',
        default_value='15.0',
        description='Transform-chain diagnostics CSV sample rate in Hz')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo /clock for launched ROS nodes')
    start_delay_sec_arg = DeclareLaunchArgument(
        'start_delay_sec',
        default_value='35.0',
        description='Delay before replay starts, after Gazebo spawn/controller startup')
    hold_initial_state_during_start_delay_arg = DeclareLaunchArgument(
        'hold_initial_state_during_start_delay',
        default_value='true',
        description=(
            'Prescribe trajectory row 0 throughout start delay so joint reactions '
            'cannot move the free Gazebo base before replay.'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'world': LaunchConfiguration('world'),
            'camera_mount_mode': LaunchConfiguration('camera_mount_mode'),
            'control_mode': 'physical_dynamics',
            'velocity_command_source': 'external',
            'camera_enabled': 'true',
            'rgbd_enabled': 'true',
            'rgbd_frame_name': 'camera_color_optical_frame',
            'rgbd_update_rate': LaunchConfiguration('rgbd_update_rate'),
            'rgbd_width': LaunchConfiguration('rgbd_width'),
            'rgbd_height': LaunchConfiguration('rgbd_height'),
            'imu_enabled': LaunchConfiguration('imu_enabled'),
            'imu_update_rate': LaunchConfiguration('imu_update_rate'),
            'use_ros2_control': 'true',
            'fix_base_to_world': 'false',
            'base_command_hold_enabled': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    baseline = Node(
        condition=baseline_condition,
        package='manipulator',
        executable='baseline_velocity_command.py',
        name='baseline_velocity_command',
        output='screen',
        arguments=[
            '--output-topic', '/arm_velocity_controller/commands',
            '--joint-count', '6',
            '--rate-hz', LaunchConfiguration('baseline_rate_hz'),
            '--use-sim-time',
        ]
    )

    apriltag = Node(
        condition=visual_condition,
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        output='screen',
        parameters=[
            LaunchConfiguration('apriltag_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('image_rect', '/d435i/color/image_raw'),
            ('camera_info', '/d435i/color/camera_info'),
        ],
    )

    visual_ee_estimator = Node(
        condition=visual_condition,
        package='manipulator',
        executable='visual_ee_pose_estimator.py',
        name='visual_ee_pose_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world_frame': 'world',
            'base_frame': 'base_link',
            'ee_frame': 'link6',
            'camera_frame': 'camera_color_optical_frame',
            'detected_tag_frame': LaunchConfiguration(
                'visual_primary_detected_tag_frame'),
            'tag_ids': LaunchConfiguration('visual_tag_ids'),
            'detected_tag_frames': LaunchConfiguration('visual_detected_tag_frames'),
            'world_to_tag_xyz': '1.600424560 0.150976374 0.200659860',
            'world_to_tag_rpy': '-3.12204785 -1.56214388 3.12103003',
            'world_to_tag_xyzs': LaunchConfiguration('visual_world_to_tag_xyzs'),
            'world_to_tag_rpys': LaunchConfiguration('visual_world_to_tag_rpys'),
            'position_estimation_mode': LaunchConfiguration(
                'visual_position_estimation_mode'),
            'detection_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_detection_timeout_sec'),
                value_type=float),
            'tf_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_tf_timeout_sec'),
                value_type=float),
            'tag_tf_mode': LaunchConfiguration('visual_tag_tf_mode'),
            'max_tag_tf_age_sec': ParameterValue(
                LaunchConfiguration('visual_max_tag_tf_age_sec'),
                value_type=float),
        }],
    )

    visual_controller = Node(
        condition=visual_condition,
        package='manipulator',
        executable='visual_ee_stabilization_controller.py',
        name='visual_ee_stabilization_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'urdf_path': arm_urdf_path,
            'ee_frame': 'link6',
            'dry_run': False,
            'control_mode': 'xyz',
            'command_topic': '/arm_velocity_controller/commands',
            'control_rate': ParameterValue(
                LaunchConfiguration('visual_stabilization_control_rate'),
                value_type=float),
            'max_joint_velocity': ParameterValue(
                LaunchConfiguration('visual_stabilization_max_joint_velocity'),
                value_type=float),
            'max_task_velocity_xyz': LaunchConfiguration(
                'visual_stabilization_max_task_velocity_xyz'),
            'position_deadband_m': ParameterValue(
                LaunchConfiguration('visual_stabilization_position_deadband_m'),
                value_type=float),
            'measurement_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_stabilization_measurement_timeout'),
                value_type=float),
            'joint_state_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_stabilization_joint_state_timeout'),
                value_type=float),
            'required_consecutive_valid_poses': ParameterValue(
                LaunchConfiguration('visual_required_consecutive_valid_poses'),
                value_type=int),
            'relock_after_visual_loss_sec': ParameterValue(
                LaunchConfiguration('visual_relock_after_visual_loss_sec'),
                value_type=float),
            'max_visual_error_norm_m': ParameterValue(
                LaunchConfiguration('visual_max_visual_error_norm_m'),
                value_type=float),
            'target_relock_enabled': ParameterValue(
                LaunchConfiguration('visual_target_relock_enabled'),
                value_type=bool),
            'stop_on_large_visual_error': ParameterValue(
                LaunchConfiguration('visual_stop_on_large_error'),
                value_type=bool),
            'max_joint_acceleration_rad_s2': ParameterValue(
                LaunchConfiguration('visual_max_joint_acceleration_rad_s2'),
                value_type=float),
        }],
    )

    ground_truth_controller = Node(
        condition=ground_truth_condition,
        package='manipulator',
        executable='ground_truth_xyz_controller.py',
        name='ground_truth_xyz_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'urdf_path': arm_urdf_path,
            'ee_frame': 'link6',
            'link_states_topic': '/link_states',
            'ee_link_name': 'windylab_arm::link6',
            'dry_run': False,
            'command_topic': '/arm_velocity_controller/commands',
            'control_rate': ParameterValue(
                LaunchConfiguration('ground_truth_control_rate'),
                value_type=float),
            'max_joint_velocity': ParameterValue(
                LaunchConfiguration('ground_truth_max_joint_velocity'),
                value_type=float),
            'max_task_velocity_xyz': LaunchConfiguration(
                'ground_truth_max_task_velocity_xyz'),
            'task_gain': LaunchConfiguration('ground_truth_task_gain'),
            'damping': ParameterValue(
                LaunchConfiguration('ground_truth_damping'),
                value_type=float),
            'position_deadband_m': ParameterValue(
                LaunchConfiguration('visual_stabilization_position_deadband_m'),
                value_type=float),
            'measurement_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_stabilization_measurement_timeout'),
                value_type=float),
            'joint_state_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_stabilization_joint_state_timeout'),
                value_type=float),
            'max_visual_error_norm_m': ParameterValue(
                LaunchConfiguration('visual_max_visual_error_norm_m'),
                value_type=float),
            'stop_on_large_error': ParameterValue(
                LaunchConfiguration('visual_stop_on_large_error'),
                value_type=bool),
            'max_joint_acceleration_rad_s2': ParameterValue(
                LaunchConfiguration('ground_truth_max_joint_acceleration_rad_s2'),
                value_type=float),
        }],
    )

    replay = Node(
        package='manipulator',
        executable='base_disturbance_replay.py',
        name='base_disturbance_replay',
        output='screen',
        arguments=[
            '--trajectory-csv', LaunchConfiguration('disturbance_csv'),
            '--output-csv', LaunchConfiguration('replay_output_csv'),
            '--rate-hz', LaunchConfiguration('replay_rate_hz'),
            '--schedule-clock', LaunchConfiguration('replay_clock_source'),
            '--state-sample-stride', LaunchConfiguration('replay_state_sample_stride'),
            '--entity-stable-samples', LaunchConfiguration('replay_entity_stable_samples'),
            '--gt-max-abs-position-m', LaunchConfiguration('replay_gt_max_abs_position_m'),
            '--service-call-timeout', LaunchConfiguration('replay_service_call_timeout'),
            '--start-delay-sec', LaunchConfiguration('start_delay_sec'),
            '--hold-initial-state-during-start-delay',
            LaunchConfiguration('hold_initial_state_during_start_delay'),
            '--camera-entity-name', LaunchConfiguration('camera_entity_name'),
            '--use-sim-time',
        ]
    )

    diagnostics = Node(
        condition=diagnostics_condition,
        package='manipulator',
        executable='phase4_visual_chain_diagnostics.py',
        name='phase4_visual_chain_diagnostics',
        output='screen',
        arguments=[
            '--output-csv', LaunchConfiguration('phase4_diagnostics_output_csv'),
            '--duration-sec', LaunchConfiguration('phase4_diagnostics_duration_sec'),
            '--sample-hz', LaunchConfiguration('phase4_diagnostics_sample_hz'),
            '--camera-frame', 'camera_color_optical_frame',
            '--detected-tag-frame', LaunchConfiguration(
                'visual_primary_detected_tag_frame'),
            '--use-sim-time',
        ],
    )

    transform_chain_diagnostics = Node(
        condition=transform_chain_diagnostics_condition,
        package='manipulator',
        executable='check_visual_pose_transform_chain.py',
        name='check_visual_pose_transform_chain',
        output='screen',
        arguments=[
            '--output-csv', LaunchConfiguration('phase4_transform_chain_output_csv'),
            '--duration-sec', LaunchConfiguration('phase4_transform_chain_duration_sec'),
            '--sample-hz', LaunchConfiguration('phase4_transform_chain_sample_hz'),
            '--camera-frame', 'camera_color_optical_frame',
            '--detected-tag-frame', LaunchConfiguration(
                'visual_primary_detected_tag_frame'),
            '--ee-frame', 'link6',
            '--base-link-name', 'windylab_arm::base_link',
            '--ee-link-name', 'windylab_arm::link6',
            '--tag-model-name', 'apriltag_36h11_board_2x2_target',
            '--tag-link-name', 'apriltag_36h11_board_2x2_target::tag_0_link',
            '--world-to-tag-xyz', '1.600424560 0.150976374 0.200659860',
            '--world-to-tag-rpy', '-3.12204785 -1.56214388 3.12103003',
            '--use-sim-time',
        ],
    )

    return LaunchDescription([
        gui_arg,
        use_rviz_arg,
        world_arg,
        apriltag_config_arg,
        camera_mount_mode_arg,
        experiment_mode_arg,
        disturbance_csv_arg,
        replay_output_csv_arg,
        camera_entity_name_arg,
        replay_rate_hz_arg,
        replay_clock_source_arg,
        replay_service_call_timeout_arg,
        replay_state_sample_stride_arg,
        replay_gt_max_abs_position_m_arg,
        replay_entity_stable_samples_arg,
        baseline_rate_hz_arg,
        visual_stabilization_control_rate_arg,
        visual_stabilization_max_joint_velocity_arg,
        visual_stabilization_max_task_velocity_xyz_arg,
        visual_stabilization_position_deadband_arg,
        visual_stabilization_measurement_timeout_arg,
        visual_stabilization_joint_state_timeout_arg,
        visual_required_consecutive_valid_poses_arg,
        visual_relock_after_visual_loss_sec_arg,
        visual_max_visual_error_norm_m_arg,
        visual_target_relock_enabled_arg,
        visual_stop_on_large_error_arg,
        visual_max_joint_acceleration_arg,
        ground_truth_control_rate_arg,
        ground_truth_max_joint_velocity_arg,
        ground_truth_max_task_velocity_xyz_arg,
        ground_truth_max_joint_acceleration_arg,
        ground_truth_task_gain_arg,
        ground_truth_damping_arg,
        visual_detection_timeout_sec_arg,
        visual_tf_timeout_sec_arg,
        visual_tag_tf_mode_arg,
        visual_max_tag_tf_age_sec_arg,
        visual_position_estimation_mode_arg,
        visual_primary_detected_tag_frame_arg,
        visual_tag_ids_arg,
        visual_detected_tag_frames_arg,
        visual_world_to_tag_xyzs_arg,
        visual_world_to_tag_rpys_arg,
        rgbd_update_rate_arg,
        rgbd_width_arg,
        rgbd_height_arg,
        imu_enabled_arg,
        imu_update_rate_arg,
        run_phase4_diagnostics_arg,
        phase4_diagnostics_output_csv_arg,
        phase4_diagnostics_duration_sec_arg,
        phase4_diagnostics_sample_hz_arg,
        run_phase4_transform_chain_diagnostics_arg,
        phase4_transform_chain_output_csv_arg,
        phase4_transform_chain_duration_sec_arg,
        phase4_transform_chain_sample_hz_arg,
        use_sim_time_arg,
        start_delay_sec_arg,
        hold_initial_state_during_start_delay_arg,
        OpaqueFunction(function=_validate_arguments),
        gazebo,
        baseline,
        apriltag,
        visual_ee_estimator,
        visual_controller,
        ground_truth_controller,
        diagnostics,
        transform_chain_diagnostics,
        replay,
    ])
