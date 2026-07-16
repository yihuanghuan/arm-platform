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
    if experiment_mode not in ('baseline', 'visual_xyz'):
        raise RuntimeError('experiment_mode must be baseline or visual_xyz')
    return []


def generate_launch_description():
    manipulator_share = FindPackageShare('manipulator').find('manipulator')
    gazebo_launch = os.path.join(manipulator_share, 'gazebo_arm.launch.py')
    default_world = os.path.join(manipulator_share, 'worlds', 'd435i_apriltag_test.world')
    apriltag_config = os.path.join(manipulator_share, 'apriltag_36h11_00000.yaml')
    arm_urdf_path = os.path.join(manipulator_share, 'arm.urdf')
    baseline_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('experiment_mode'), "' == 'baseline'"]))
    visual_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration('experiment_mode'), "' == 'visual_xyz'"]))

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
    camera_mount_mode_arg = DeclareLaunchArgument(
        'camera_mount_mode',
        default_value='ee',
        description='D435i mount preset: ee or base')
    experiment_mode_arg = DeclareLaunchArgument(
        'experiment_mode',
        default_value='baseline',
        description='Experiment mode: baseline or visual_xyz')
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
    replay_service_call_timeout_arg = DeclareLaunchArgument(
        'replay_service_call_timeout',
        default_value='0.2',
        description='Timeout for each Gazebo set/get entity state service call')
    replay_state_sample_stride_arg = DeclareLaunchArgument(
        'replay_state_sample_stride',
        default_value='1',
        description='Query Gazebo entity states once every N replay samples')
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
        default_value='1.0',
        description='Visual XYZ joint velocity clamp in rad/s')
    visual_stabilization_max_task_velocity_xyz_arg = DeclareLaunchArgument(
        'visual_stabilization_max_task_velocity_xyz',
        default_value='0.25 0.25 0.25',
        description='Visual XYZ task velocity clamp in m/s')
    visual_stabilization_position_deadband_arg = DeclareLaunchArgument(
        'visual_stabilization_position_deadband_m',
        default_value='0.003',
        description='Visual XYZ position deadband in meters')
    visual_stabilization_measurement_timeout_arg = DeclareLaunchArgument(
        'visual_stabilization_measurement_timeout',
        default_value='5.0',
        description='Maximum visual measurement age before zero velocity')
    visual_stabilization_joint_state_timeout_arg = DeclareLaunchArgument(
        'visual_stabilization_joint_state_timeout',
        default_value='0.35',
        description='Maximum joint-state age before zero velocity')
    visual_detection_timeout_sec_arg = DeclareLaunchArgument(
        'visual_detection_timeout_sec',
        default_value='5.0',
        description='Maximum AprilTag detection age before visual pose invalid')
    visual_tf_timeout_sec_arg = DeclareLaunchArgument(
        'visual_tf_timeout_sec',
        default_value='0.1',
        description='TF lookup timeout for visual end-effector pose estimation')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo /clock for launched ROS nodes')
    start_delay_sec_arg = DeclareLaunchArgument(
        'start_delay_sec',
        default_value='12.0',
        description='Delay before replay starts, after Gazebo spawn/controller startup')

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
            'imu_enabled': 'true',
            'use_ros2_control': 'true',
            'fix_base_to_world': 'false',
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
        parameters=[apriltag_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
            'detected_tag_frame': 'apriltag_36h11_00000',
            'world_to_tag_xyz': '1.23042456 0.000976374 0.35065986',
            'world_to_tag_rpy': '-3.12204785 -1.56214388 3.12103003',
            'position_estimation_mode': 'kinematic_orientation',
            'detection_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_detection_timeout_sec'),
                value_type=float),
            'tf_timeout_sec': ParameterValue(
                LaunchConfiguration('visual_tf_timeout_sec'),
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
            '--state-sample-stride', LaunchConfiguration('replay_state_sample_stride'),
            '--service-call-timeout', LaunchConfiguration('replay_service_call_timeout'),
            '--start-delay-sec', LaunchConfiguration('start_delay_sec'),
            '--camera-entity-name', LaunchConfiguration('camera_entity_name'),
            '--use-sim-time',
        ]
    )

    return LaunchDescription([
        gui_arg,
        use_rviz_arg,
        world_arg,
        camera_mount_mode_arg,
        experiment_mode_arg,
        disturbance_csv_arg,
        replay_output_csv_arg,
        camera_entity_name_arg,
        replay_rate_hz_arg,
        replay_service_call_timeout_arg,
        replay_state_sample_stride_arg,
        baseline_rate_hz_arg,
        visual_stabilization_control_rate_arg,
        visual_stabilization_max_joint_velocity_arg,
        visual_stabilization_max_task_velocity_xyz_arg,
        visual_stabilization_position_deadband_arg,
        visual_stabilization_measurement_timeout_arg,
        visual_stabilization_joint_state_timeout_arg,
        visual_detection_timeout_sec_arg,
        visual_tf_timeout_sec_arg,
        use_sim_time_arg,
        start_delay_sec_arg,
        OpaqueFunction(function=_validate_arguments),
        gazebo,
        baseline,
        apriltag,
        visual_ee_estimator,
        visual_controller,
        replay,
    ])
