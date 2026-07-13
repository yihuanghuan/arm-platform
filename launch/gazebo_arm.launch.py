from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xml.etree.ElementTree as ET
import xacro


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _auto_or_empty(value):
    return str(value).strip().lower() in ('', 'auto')


def _resolve_camera_mount(context):
    mount_mode = LaunchConfiguration('camera_mount_mode').perform(context)
    if mount_mode not in ('ee', 'base'):
        raise RuntimeError('camera_mount_mode must be one of: ee, base')

    if mount_mode == 'base':
        default_parent_link = 'base_link'
        default_xyz = '0.02 0 0.06'
        default_rpy = '0 0 0'
    else:
        default_parent_link = 'link6'
        default_xyz = '0.06 0 0.04'
        default_rpy = '0 0 0'

    camera_parent_link = LaunchConfiguration('camera_parent_link').perform(context)
    camera_xyz = LaunchConfiguration('camera_xyz').perform(context)
    camera_rpy = LaunchConfiguration('camera_rpy').perform(context)

    return {
        'camera_mount_mode': mount_mode,
        'camera_parent_link': (
            default_parent_link if _auto_or_empty(camera_parent_link) else camera_parent_link),
        'camera_xyz': default_xyz if _auto_or_empty(camera_xyz) else camera_xyz,
        'camera_rpy': default_rpy if _auto_or_empty(camera_rpy) else camera_rpy,
    }


def _render_robot_description(pkg_share, context):
    xacro_path = os.path.join(pkg_share, 'arm_with_d435i.urdf.xacro')
    camera_mount = _resolve_camera_mount(context)
    doc = xacro.process_file(
        xacro_path,
        mappings={
            'camera_enabled': LaunchConfiguration('camera_enabled').perform(context),
            'camera_mount_mode': camera_mount['camera_mount_mode'],
            'camera_name': LaunchConfiguration('camera_name').perform(context),
            'camera_parent_link': camera_mount['camera_parent_link'],
            'camera_xyz': camera_mount['camera_xyz'],
            'camera_rpy': camera_mount['camera_rpy'],
            'camera_use_nominal_extrinsics': LaunchConfiguration(
                'camera_use_nominal_extrinsics').perform(context),
            'rgbd_enabled': LaunchConfiguration('rgbd_enabled').perform(context),
            'rgbd_namespace': LaunchConfiguration('rgbd_namespace').perform(context),
            'rgbd_camera_name': LaunchConfiguration('rgbd_camera_name').perform(context),
            'rgbd_frame_name': LaunchConfiguration('rgbd_frame_name').perform(context),
            'rgbd_update_rate': LaunchConfiguration('rgbd_update_rate').perform(context),
            'rgbd_width': LaunchConfiguration('rgbd_width').perform(context),
            'rgbd_height': LaunchConfiguration('rgbd_height').perform(context),
            'rgbd_horizontal_fov': LaunchConfiguration('rgbd_horizontal_fov').perform(context),
            'rgbd_near': LaunchConfiguration('rgbd_near').perform(context),
            'rgbd_far': LaunchConfiguration('rgbd_far').perform(context),
            'rgbd_visualize': LaunchConfiguration('rgbd_visualize').perform(context),
            'imu_enabled': LaunchConfiguration('imu_enabled').perform(context),
            'imu_namespace': LaunchConfiguration('imu_namespace').perform(context),
            'imu_topic': LaunchConfiguration('imu_topic').perform(context),
            'imu_frame_name': LaunchConfiguration('imu_frame_name').perform(context),
            'imu_update_rate': LaunchConfiguration('imu_update_rate').perform(context),
            'imu_visualize': LaunchConfiguration('imu_visualize').perform(context),
            'imu_noise_mean': LaunchConfiguration('imu_noise_mean').perform(context),
            'imu_angular_velocity_noise_stddev': LaunchConfiguration(
                'imu_angular_velocity_noise_stddev').perform(context),
            'imu_linear_acceleration_noise_stddev': LaunchConfiguration(
                'imu_linear_acceleration_noise_stddev').perform(context),
            'use_ros2_control': LaunchConfiguration('use_ros2_control').perform(context),
            'fix_base_to_world': LaunchConfiguration('fix_base_to_world').perform(context),
        })
    robot_description = doc.toprettyxml(indent='  ')

    if robot_description.startswith('<?xml'):
        robot_description = robot_description.split('\n', 1)[1]
    return robot_description


def _strip_collision_elements(robot_description):
    root = ET.fromstring(robot_description)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'collision':
                parent.remove(child)
    return ET.tostring(root, encoding='unicode')


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    dummy_description_share = FindPackageShare('dummy_description').find('dummy_description')
    realsense_description_share = FindPackageShare('realsense2_description').find(
        'realsense2_description')
    robot_description = _render_robot_description(pkg_share, context)
    use_ros2_control = _as_bool(LaunchConfiguration('use_ros2_control').perform(context))
    static_model = _as_bool(LaunchConfiguration('static_model').perform(context))
    use_rviz = _as_bool(LaunchConfiguration('use_rviz').perform(context))
    disable_collisions = _as_bool(LaunchConfiguration('disable_collisions').perform(context))
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    control_mode = LaunchConfiguration('control_mode').perform(context)
    valid_control_modes = ('kinematic_visualization', 'physical_dynamics')
    if control_mode not in valid_control_modes:
        raise RuntimeError(
            'control_mode must be one of: ' + ', '.join(valid_control_modes))
    velocity_command_source = LaunchConfiguration('velocity_command_source').perform(context)
    valid_velocity_command_sources = ('student_bridge', 'external')
    if velocity_command_source not in valid_velocity_command_sources:
        raise RuntimeError(
            'velocity_command_source must be one of: '
            + ', '.join(valid_velocity_command_sources))

    mesh_rewrites = {
        'package://dummy_description/': 'file://' + dummy_description_share + '/',
        'model://dummy_description/': 'file://' + dummy_description_share + '/',
        'package://realsense2_description/': 'file://' + realsense_description_share + '/',
        'model://realsense2_description/': 'file://' + realsense_description_share + '/',
    }
    for old, new in mesh_rewrites.items():
        robot_description = robot_description.replace(old, new)

    if disable_collisions:
        robot_description = _strip_collision_elements(robot_description)

    if not use_ros2_control and static_model:
        robot_description = robot_description.replace(
            '</robot>',
            '  <gazebo>\n'
            '    <static>true</static>\n'
            '  </gazebo>\n'
            '</robot>',
            1)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    spawn_arm = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_arm',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'windylab_arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ]
    )

    actions = [robot_state_publisher, spawn_arm]

    if use_ros2_control:
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_joint_state_broadcaster',
            output='screen',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
            ]
        )

        controller_actions = [joint_state_broadcaster_spawner]

        if control_mode == 'physical_dynamics':
            arm_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_arm_velocity_controller',
                output='screen',
                arguments=[
                    'arm_velocity_controller',
                    '--controller-manager', '/controller_manager',
                ]
            )

            command_bridge = None
            if velocity_command_source == 'student_bridge':
                command_bridge = Node(
                    package='manipulator',
                    executable='student_joint_velocity_bridge.py',
                    name='student_joint_velocity_bridge',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'input_topic': '/student/joint_command',
                        'joint_state_topic': '/joint_states',
                        'controller_command_topic': '/arm_velocity_controller/commands',
                        'joint_names': [
                            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
                        'kp': float(LaunchConfiguration('velocity_kp').perform(context)),
                        'feedforward_scale': float(
                            LaunchConfiguration('velocity_feedforward_scale').perform(context)),
                        'max_velocity': float(
                            LaunchConfiguration('velocity_max_velocity').perform(context)),
                        'position_tolerance': float(
                            LaunchConfiguration('velocity_position_tolerance').perform(context)),
                        'publish_rate_hz': float(
                            LaunchConfiguration('velocity_publish_rate').perform(context)),
                        'command_timeout_sec': float(
                            LaunchConfiguration('velocity_command_timeout_sec').perform(context)),
                    }]
                )
        else:
            arm_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_arm_position_controller',
                output='screen',
                arguments=[
                    'arm_position_controller',
                    '--controller-manager', '/controller_manager',
                ]
            )

            command_bridge = Node(
                package='manipulator',
                executable='student_joint_command_bridge.py',
                name='student_joint_command_bridge',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic': '/student/joint_command',
                    'controller_command_topic': '/arm_position_controller/commands',
                    'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
                }]
            )

        controller_actions.append(arm_controller_spawner)
        if command_bridge is not None:
            controller_actions.append(command_bridge)

        actions.append(TimerAction(
            period=3.0,
            actions=controller_actions))

    if use_rviz:
        rviz_config = os.path.join(pkg_share, 'student_arm.rviz')
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ))

    return actions


def generate_launch_description():
    gazebo_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    manipulator_share = FindPackageShare('manipulator').find('manipulator')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_paths = [
        '/usr/share/gazebo-11/models',
        os.path.join(manipulator_share, 'models'),
    ]
    if gazebo_model_path:
        model_paths.append(gazebo_model_path)

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo Classic GUI client'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose Gazebo server output'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=[FindPackageShare('gazebo_ros'), '/worlds/empty.world'],
        description='Gazebo world file'
    )

    camera_enabled_arg = DeclareLaunchArgument(
        'camera_enabled',
        default_value='true',
        description='Attach the D435i description to robot_description'
    )

    camera_mount_mode_arg = DeclareLaunchArgument(
        'camera_mount_mode',
        default_value='ee',
        description='D435i mount preset: ee or base'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='D435i link and frame name prefix'
    )

    camera_parent_link_arg = DeclareLaunchArgument(
        'camera_parent_link',
        default_value='auto',
        description='Parent link for the D435i mount, or auto for camera_mount_mode default'
    )

    camera_xyz_arg = DeclareLaunchArgument(
        'camera_xyz',
        default_value='auto',
        description='D435i mount translation relative to camera_parent_link, or auto'
    )

    camera_rpy_arg = DeclareLaunchArgument(
        'camera_rpy',
        default_value='auto',
        description='D435i mount orientation relative to camera_parent_link, or auto'
    )

    camera_use_nominal_extrinsics_arg = DeclareLaunchArgument(
        'camera_use_nominal_extrinsics',
        default_value='true',
        description='Use official D435i nominal camera and IMU extrinsic frames'
    )

    rgbd_enabled_arg = DeclareLaunchArgument(
        'rgbd_enabled',
        default_value='true',
        description='Attach the Gazebo RGB-D sensor plugin to the D435i body'
    )

    rgbd_namespace_arg = DeclareLaunchArgument(
        'rgbd_namespace',
        default_value='d435i',
        description='ROS namespace for RGB-D camera topics'
    )

    rgbd_camera_name_arg = DeclareLaunchArgument(
        'rgbd_camera_name',
        default_value='color',
        description='gazebo_ros_camera camera_name used for color image topics'
    )

    rgbd_frame_name_arg = DeclareLaunchArgument(
        'rgbd_frame_name',
        default_value='camera_depth_optical_frame',
        description='Header frame_id used by the Gazebo RGB-D plugin'
    )

    rgbd_update_rate_arg = DeclareLaunchArgument(
        'rgbd_update_rate',
        default_value='15',
        description='RGB-D sensor update rate in Hz'
    )

    rgbd_width_arg = DeclareLaunchArgument(
        'rgbd_width',
        default_value='640',
        description='RGB-D image width'
    )

    rgbd_height_arg = DeclareLaunchArgument(
        'rgbd_height',
        default_value='480',
        description='RGB-D image height'
    )

    rgbd_horizontal_fov_arg = DeclareLaunchArgument(
        'rgbd_horizontal_fov',
        default_value='1.211',
        description='RGB-D horizontal field of view in radians'
    )

    rgbd_near_arg = DeclareLaunchArgument(
        'rgbd_near',
        default_value='0.1',
        description='RGB-D near clipping and minimum valid depth in meters'
    )

    rgbd_far_arg = DeclareLaunchArgument(
        'rgbd_far',
        default_value='5.0',
        description='RGB-D far clipping and maximum valid depth in meters'
    )

    rgbd_visualize_arg = DeclareLaunchArgument(
        'rgbd_visualize',
        default_value='false',
        description='Show the Gazebo RGB-D camera frustum'
    )

    imu_enabled_arg = DeclareLaunchArgument(
        'imu_enabled',
        default_value='true',
        description='Attach the Gazebo IMU sensor plugin to the D435i body'
    )

    imu_namespace_arg = DeclareLaunchArgument(
        'imu_namespace',
        default_value='d435i',
        description='ROS namespace for D435i IMU topics'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='imu',
        description='IMU topic name inside imu_namespace'
    )

    imu_frame_name_arg = DeclareLaunchArgument(
        'imu_frame_name',
        default_value='camera_accel_optical_frame',
        description='Header frame_id used by the Gazebo IMU plugin'
    )

    imu_update_rate_arg = DeclareLaunchArgument(
        'imu_update_rate',
        default_value='200',
        description='IMU sensor update rate in Hz'
    )

    imu_visualize_arg = DeclareLaunchArgument(
        'imu_visualize',
        default_value='false',
        description='Show the Gazebo IMU sensor visualization'
    )

    imu_noise_mean_arg = DeclareLaunchArgument(
        'imu_noise_mean',
        default_value='0.0',
        description='Mean for Gazebo IMU Gaussian noise'
    )

    imu_angular_velocity_noise_stddev_arg = DeclareLaunchArgument(
        'imu_angular_velocity_noise_stddev',
        default_value='0.0',
        description='Angular velocity Gaussian noise stddev in rad/s'
    )

    imu_linear_acceleration_noise_stddev_arg = DeclareLaunchArgument(
        'imu_linear_acceleration_noise_stddev',
        default_value='0.0',
        description='Linear acceleration Gaussian noise stddev in m/s^2'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Load gazebo_ros2_control and drive the Gazebo joints from ROS 2'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='kinematic_visualization',
        description='Gazebo control mode: kinematic_visualization or physical_dynamics'
    )

    velocity_kp_arg = DeclareLaunchArgument(
        'velocity_kp',
        default_value='4.0',
        description='Proportional gain from joint position error to velocity command'
    )

    velocity_command_source_arg = DeclareLaunchArgument(
        'velocity_command_source',
        default_value='student_bridge',
        description=(
            'Velocity command publisher in physical_dynamics mode: '
            'student_bridge or external')
    )

    velocity_feedforward_scale_arg = DeclareLaunchArgument(
        'velocity_feedforward_scale',
        default_value='1.0',
        description='Scale applied to /student/joint_command.velocity in physical_dynamics mode'
    )

    velocity_max_velocity_arg = DeclareLaunchArgument(
        'velocity_max_velocity',
        default_value='1.0',
        description='Absolute velocity command limit in rad/s for physical_dynamics mode'
    )

    velocity_position_tolerance_arg = DeclareLaunchArgument(
        'velocity_position_tolerance',
        default_value='0.005',
        description='Position error tolerance in rad below which velocity command is zeroed'
    )

    velocity_publish_rate_arg = DeclareLaunchArgument(
        'velocity_publish_rate',
        default_value='100.0',
        description='Velocity bridge command publish rate in Hz'
    )

    velocity_command_timeout_sec_arg = DeclareLaunchArgument(
        'velocity_command_timeout_sec',
        default_value='0.25',
        description='After this timeout, ignore stale feedforward velocity and hold target position'
    )

    fix_base_to_world_arg = DeclareLaunchArgument(
        'fix_base_to_world',
        default_value='true',
        description='Add a fixed world_to_base_link joint for Gazebo simulation'
    )

    static_model_arg = DeclareLaunchArgument(
        'static_model',
        default_value='true',
        description='Use a static Gazebo model only when use_ros2_control is false'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz with the student arm configuration'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo /clock for ROS nodes launched here'
    )

    disable_collisions_arg = DeclareLaunchArgument(
        'disable_collisions',
        default_value='true',
        description='Remove collision geometry from the Gazebo robot model'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
            'world': LaunchConfiguration('world'),
            'factory': 'true',
        }.items()
    )

    return LaunchDescription([
        gui_arg,
        verbose_arg,
        world_arg,
        camera_enabled_arg,
        camera_mount_mode_arg,
        camera_name_arg,
        camera_parent_link_arg,
        camera_xyz_arg,
        camera_rpy_arg,
        camera_use_nominal_extrinsics_arg,
        rgbd_enabled_arg,
        rgbd_namespace_arg,
        rgbd_camera_name_arg,
        rgbd_frame_name_arg,
        rgbd_update_rate_arg,
        rgbd_width_arg,
        rgbd_height_arg,
        rgbd_horizontal_fov_arg,
        rgbd_near_arg,
        rgbd_far_arg,
        rgbd_visualize_arg,
        imu_enabled_arg,
        imu_namespace_arg,
        imu_topic_arg,
        imu_frame_name_arg,
        imu_update_rate_arg,
        imu_visualize_arg,
        imu_noise_mean_arg,
        imu_angular_velocity_noise_stddev_arg,
        imu_linear_acceleration_noise_stddev_arg,
        use_ros2_control_arg,
        control_mode_arg,
        velocity_command_source_arg,
        velocity_kp_arg,
        velocity_feedforward_scale_arg,
        velocity_max_velocity_arg,
        velocity_position_tolerance_arg,
        velocity_publish_rate_arg,
        velocity_command_timeout_sec_arg,
        fix_base_to_world_arg,
        static_model_arg,
        use_rviz_arg,
        use_sim_time_arg,
        disable_collisions_arg,
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_paths)),
        gazebo,
        OpaqueFunction(function=_launch_setup),
    ])
