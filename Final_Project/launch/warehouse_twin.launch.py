from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动三个节点：模拟器、控制器、可视化"""
    
    return LaunchDescription([
        # 1. 机器人模拟器（物理计算）
        Node(
            package='warehouse_digital_twin',
            executable='simulator',
            name='robot_simulator',
            output='screen'
        ),
        # 2. 控制器（导航 + 任务调度）
        Node(
            package='warehouse_digital_twin',
            executable='controller',
            name='warehouse_controller',
            output='screen'
        ),
        # 3. 可视化界面（画图显示）
        Node(
            package='warehouse_digital_twin',
            executable='visualizer',
            name='visualizer',
            output='screen'
        ),
    ])
