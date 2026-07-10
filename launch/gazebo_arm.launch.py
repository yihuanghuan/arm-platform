from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('manipulator').find('manipulator')
    gazebo_share = FindPackageShare('gazebo_ros').find('gazebo_ros')

    urdf_path = os.path.join(pkg_share, 'arm.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    if robot_description.startswith('<?xml'):
        robot_description = robot_description.split('\n', 1)[1]

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

    return LaunchDescription([
        gui_arg,
        verbose_arg,
        gazebo,
        robot_state_publisher,
        spawn_arm,
    ])
