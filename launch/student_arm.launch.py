from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
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


def _robot_description(pkg_share, context):
    camera_enabled = _as_bool(LaunchConfiguration('camera_enabled').perform(context))
    if not camera_enabled:
        urdf_path = os.path.join(pkg_share, 'arm.urdf')
        with open(urdf_path, 'r') as f:
            return f.read()

    xacro_path = os.path.join(pkg_share, 'arm_with_d435i.urdf.xacro')
    camera_mount = _resolve_camera_mount(context)
    doc = xacro.process_file(
        xacro_path,
        mappings={
            'camera_enabled': 'true',
            'camera_mount_mode': camera_mount['camera_mount_mode'],
            'camera_name': LaunchConfiguration('camera_name').perform(context),
            'camera_parent_link': camera_mount['camera_parent_link'],
            'camera_xyz': camera_mount['camera_xyz'],
            'camera_rpy': camera_mount['camera_rpy'],
            'camera_use_nominal_extrinsics': LaunchConfiguration(
                'camera_use_nominal_extrinsics').perform(context),
        })
    return doc.toprettyxml(indent='  ')


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    robot_description = _robot_description(pkg_share, context)
    rviz_config_path = os.path.join(pkg_share, 'student_arm.rviz')
    motor_config_path = os.path.join(pkg_share, 'motor_config.yaml')
    arm_config_path = os.path.join(pkg_share, 'arm_config.yaml')

    student_arm_node = Node(
        package='manipulator',
        executable='student_arm_node',
        name='student_arm_node',
        output='screen',
        parameters=[
            {'arm_type': LaunchConfiguration('arm_type')},
            {'arm_version': 'gamma'},
            {'port_name': LaunchConfiguration('port_name')},
            {'motor_config_path': motor_config_path},
            {'arm_config_path': arm_config_path},
            {'max_velocity': LaunchConfiguration('max_velocity')},
            {'command_timeout_sec': 1.0},
            {'publish_joint_feedback': LaunchConfiguration('publish_joint_feedback')},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return [student_arm_node, robot_state_publisher_node, rviz_node]


def generate_launch_description():
    # 机械臂类型: sim(虚拟臂, 默认) 或 a_l1(真实硬件)
    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='sim',
        description='Arm type: sim (virtual) or a_l1 (real hardware)'
    )

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyTHS3',
        description='Serial port for real hardware (ignored in sim mode)'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='0.5',
        description='Max joint velocity (rad/s) safety limit for students'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz for visualization'
    )

    publish_joint_feedback_arg = DeclareLaunchArgument(
        'publish_joint_feedback',
        default_value='false',
        description='Publish /student/joint_feedback when true'
    )

    camera_enabled_arg = DeclareLaunchArgument(
        'camera_enabled',
        default_value='false',
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

    return LaunchDescription([
        arm_type_arg,
        port_name_arg,
        max_velocity_arg,
        use_rviz_arg,
        publish_joint_feedback_arg,
        camera_enabled_arg,
        camera_mount_mode_arg,
        camera_name_arg,
        camera_parent_link_arg,
        camera_xyz_arg,
        camera_rpy_arg,
        camera_use_nominal_extrinsics_arg,
        OpaqueFunction(function=_launch_setup),
    ])
