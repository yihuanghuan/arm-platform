from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def _validate_arguments(context, *args, **kwargs):
    experiment_mode = LaunchConfiguration('experiment_mode').perform(context)
    if experiment_mode != 'baseline':
        raise RuntimeError('Stage 8.0 only supports experiment_mode:=baseline')
    return []


def generate_launch_description():
    manipulator_share = FindPackageShare('manipulator').find('manipulator')
    gazebo_launch = os.path.join(manipulator_share, 'gazebo_arm.launch.py')
    default_world = os.path.join(manipulator_share, 'worlds', 'd435i_apriltag_test.world')

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
        description='Experiment mode. Stage 8.0 supports baseline.')
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
    baseline_rate_hz_arg = DeclareLaunchArgument(
        'baseline_rate_hz',
        default_value='100.0',
        description='Baseline zero velocity command rate')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo /clock for launched ROS nodes')
    start_delay_sec_arg = DeclareLaunchArgument(
        'start_delay_sec',
        default_value='4.0',
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

    replay = Node(
        package='manipulator',
        executable='base_disturbance_replay.py',
        name='base_disturbance_replay',
        output='screen',
        arguments=[
            '--trajectory-csv', LaunchConfiguration('disturbance_csv'),
            '--output-csv', LaunchConfiguration('replay_output_csv'),
            '--rate-hz', LaunchConfiguration('replay_rate_hz'),
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
        baseline_rate_hz_arg,
        use_sim_time_arg,
        start_delay_sec_arg,
        OpaqueFunction(function=_validate_arguments),
        gazebo,
        baseline,
        replay,
    ])
