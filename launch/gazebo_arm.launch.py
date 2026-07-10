from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _render_robot_description(pkg_share, context):
    camera_enabled = _as_bool(LaunchConfiguration('camera_enabled').perform(context))
    if camera_enabled:
        xacro_path = os.path.join(pkg_share, 'arm_with_d435i.urdf.xacro')
        doc = xacro.process_file(
            xacro_path,
            mappings={
                'camera_enabled': 'true',
                'camera_name': LaunchConfiguration('camera_name').perform(context),
                'camera_parent_link': LaunchConfiguration('camera_parent_link').perform(context),
                'camera_xyz': LaunchConfiguration('camera_xyz').perform(context),
                'camera_rpy': LaunchConfiguration('camera_rpy').perform(context),
                'camera_use_nominal_extrinsics': LaunchConfiguration(
                    'camera_use_nominal_extrinsics').perform(context),
            })
        robot_description = doc.toprettyxml(indent='  ')
    else:
        urdf_path = os.path.join(pkg_share, 'arm.urdf')
        with open(urdf_path, 'r') as f:
            robot_description = f.read()

    if robot_description.startswith('<?xml'):
        robot_description = robot_description.split('\n', 1)[1]
    return robot_description


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    dummy_description_share = FindPackageShare('dummy_description').find('dummy_description')
    realsense_description_share = FindPackageShare('realsense2_description').find(
        'realsense2_description')
    robot_description = _render_robot_description(pkg_share, context)

    mesh_rewrites = {
        'package://dummy_description/': 'file://' + dummy_description_share + '/',
        'model://dummy_description/': 'file://' + dummy_description_share + '/',
        'package://realsense2_description/': 'file://' + realsense_description_share + '/',
        'model://realsense2_description/': 'file://' + realsense_description_share + '/',
    }
    for old, new in mesh_rewrites.items():
        robot_description = robot_description.replace(old, new)

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

    return [robot_state_publisher, spawn_arm]


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
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_paths)),
        gazebo,
        OpaqueFunction(function=_launch_setup),
    ])
