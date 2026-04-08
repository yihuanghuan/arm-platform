from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Node(package='ti5_arm', executable='ti5_arm_node', name='robot_arm',
        #      parameters=[{'port_name': '/dev/ttyUSB0'}]),
        # ExecuteProcess(
        #     cmd=[
        #         "gnome-terminal", "--",
        #         "ros2", "run", "dummy_arm", "dummy_arm_node",
        #         "--ros-args",
        #         "-p", "port_name:=/dev/ttyUSB0"
        #     ],
        #     output="screen"
        # ),
        Node(
            package='manipulator',
            executable='arm_hardware_node',
            name='arm_hardware_node',
            output='screen',
            parameters=[
                {'port_name': '/dev/ttyUSB0'},
                {'arm_type': 'a_l1_beta'},
            ],
        ),
        # Node(
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     name='v4l2_camera_node',
        #     output='screen',
        #     parameters=[
        #         {'video_device': '/dev/video1'},
        #         {'image_width': 640},
        #         {'image_height': 480},
        #         {'framerate': 30}
        #     ]
        # ),
    ])
