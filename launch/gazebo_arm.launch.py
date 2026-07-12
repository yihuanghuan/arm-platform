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


def _render_robot_description(pkg_share, context):
    xacro_path = os.path.join(pkg_share, 'arm_with_d435i.urdf.xacro')
    doc = xacro.process_file(
        xacro_path,
        mappings={
            'camera_enabled': LaunchConfiguration('camera_enabled').perform(context),
            'camera_name': LaunchConfiguration('camera_name').perform(context),
            'camera_parent_link': LaunchConfiguration('camera_parent_link').perform(context),
            'camera_xyz': LaunchConfiguration('camera_xyz').perform(context),
            'camera_rpy': LaunchConfiguration('camera_rpy').perform(context),
            'camera_use_nominal_extrinsics': LaunchConfiguration(
                'camera_use_nominal_extrinsics').perform(context),
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
        parameters=[{'robot_description': robot_description}]
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

        arm_position_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_arm_position_controller',
            output='screen',
            arguments=[
                'arm_position_controller',
                '--controller-manager', '/controller_manager',
            ]
        )

        student_joint_command_bridge = Node(
            package='manipulator',
            executable='student_joint_command_bridge.py',
            name='student_joint_command_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/student/joint_command',
                'controller_command_topic': '/arm_position_controller/commands',
                'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            }]
        )

        actions.append(TimerAction(
            period=3.0,
            actions=[
                joint_state_broadcaster_spawner,
                arm_position_controller_spawner,
                student_joint_command_bridge,
            ]))

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
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_paths = [
        '/usr/share/gazebo-11/models',
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

    camera_enabled_arg = DeclareLaunchArgument(
        'camera_enabled',
        default_value='true',
        description='Attach the D435i description to robot_description'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='D435i link and frame name prefix'
    )

    camera_parent_link_arg = DeclareLaunchArgument(
        'camera_parent_link',
        default_value='link6',
        description='Parent link for the D435i mount'
    )

    camera_xyz_arg = DeclareLaunchArgument(
        'camera_xyz',
        default_value='0.06 0 0.04',
        description='D435i mount translation relative to camera_parent_link'
    )

    camera_rpy_arg = DeclareLaunchArgument(
        'camera_rpy',
        default_value='0 0 0',
        description='D435i mount orientation relative to camera_parent_link'
    )

    camera_use_nominal_extrinsics_arg = DeclareLaunchArgument(
        'camera_use_nominal_extrinsics',
        default_value='true',
        description='Use official D435i nominal camera and IMU extrinsic frames'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Load gazebo_ros2_control and drive the Gazebo joints from ROS 2'
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
            'factory': 'true',
        }.items()
    )

    return LaunchDescription([
        gui_arg,
        verbose_arg,
        camera_enabled_arg,
        camera_name_arg,
        camera_parent_link_arg,
        camera_xyz_arg,
        camera_rpy_arg,
        camera_use_nominal_extrinsics_arg,
        use_ros2_control_arg,
        fix_base_to_world_arg,
        static_model_arg,
        use_rviz_arg,
        disable_collisions_arg,
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_paths)),
        gazebo,
        OpaqueFunction(function=_launch_setup),
    ])
