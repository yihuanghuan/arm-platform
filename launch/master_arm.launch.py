from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    publish_joint_state_arg = DeclareLaunchArgument(
        'publish_joint_state',
        default_value='False',
        description='Whether to publish joint states'
    )
    
    publish_joint_feedback_arg = DeclareLaunchArgument(
        'publish_joint_feedback',
        default_value='False',
        description='Whether to publish joint feedback'
    )
    
    return LaunchDescription([
        publish_joint_state_arg,
        publish_joint_feedback_arg,
        Node(
            package='manipulator',
            executable='master_arm_node',
            name='master_arm_node',
            output='screen',
            namespace='master',
            parameters=[
                {'arm_type': 'a_l1_gamma'},
                # 串口端口名
                {'port_name': '/dev/ttyUSB0'},
                # 关节增益参数（0/1/2轴）
                {'G_GAIN_0': 0.9},
                {'G_GAIN_1': 0.5},
                {'G_GAIN_2': 1.1},
                # 最大扭矩限制
                {'MAX_TORQUE': 3.0},
                # 重力加速度
                {'GRAVITY': 9.81},
                # 无人机姿态补偿（roll/pitch/yaw）
                {'uav_roll': 0.0},
                {'uav_pitch': 0.0},
                {'uav_yaw': 0.0},
                # 机械臂旋转角度（弧度制，roll/pitch/yaw）
                {'arm_roll': 0.0},
                {'arm_pitch': 0.0},
                {'arm_yaw': 0.0},
                # 调试信息开关
                {'debug_info': True},
                # 调试信息打印频率（Hz）
                {'debug_rate': 1.0},
                # 力反馈阈值
                {'FORCE_FEEDBACK_THRESHOLD': 0.5},
                # 力反馈增益
                {'FORCE_FEEDBACK_GAIN': 0.5},
                # 是否发布joint_state
                {'publish_joint_state': LaunchConfiguration('publish_joint_state')},
                # 是否发布joint_feedback
                {'publish_joint_feedback': LaunchConfiguration('publish_joint_feedback')},
                {'urdf_path': "/home/iusl/huaben_ws/src/dummy_description/urdf/arm/A-L1-GAMMA/arm.urdf"}
            ]
        ),
    ])
