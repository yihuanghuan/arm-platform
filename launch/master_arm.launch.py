from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
    
    master_arm_node = Node(
        package='manipulator',
        executable='master_arm_node',
        name='master_arm_node',
        output='screen',
        namespace='master',
        sigterm_timeout='20',
        sigkill_timeout='20',
        parameters=[
            {'arm_type': 'a_l1_gamma'},
            {'auto_reset': False},
            # 串口端口名
            {'port_name': '/dev/ttyUSB0'},        
            # 最大扭矩限制
            {'MAX_TORQUE': 3.0},
            # 重力加速度
            {'GRAVITY': 9.81},
            # 无人机姿态补偿（roll/pitch/yaw）
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
            {'urdf_path': PathJoinSubstitution([FindPackageShare('manipulator'), 'arm.urdf'])}
        ]
    )
    
    return LaunchDescription([
        publish_joint_state_arg,
        publish_joint_feedback_arg,
        master_arm_node,
    ])
