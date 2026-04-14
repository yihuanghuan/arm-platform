from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator',
            executable='slave_arm_node',
            name='slave_arm_node',
            output='screen',
            namespace='/slave',
            parameters=[
                {'arm_type': 'a_l1_gamma'},
                # 串口端口名
                {'port_name': '/dev/ttyUSB0'},
                # 关节增益参数（0/1/2轴）
                {'G_GAIN_0': 0.5},
                {'G_GAIN_1': 0.5},
                {'G_GAIN_2': 1.0},
                # 最大扭矩限制
                {'MAX_TORQUE': 3.0},
                # 重力加速度
                {'GRAVITY': 9.81},
                # 力反馈阈值
                {'FORCE_FEEDBACK_THRESHOLD': 0.5},
                # 力反馈增益
                {'FORCE_FEEDBACK_GAIN': 0.5},
                # 调试信息开关
                {'debug_info': True},
                # 调试信息打印频率（Hz）
                {'debug_rate': 1.0},
                # 是否发布 joint states
                {'publish_joint_states': True},
                {'urdf_path': "/home/crz/arm-platform/arm.urdf"}
            ]
        ),
    ])
