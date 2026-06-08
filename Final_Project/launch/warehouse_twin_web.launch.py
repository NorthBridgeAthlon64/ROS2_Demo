from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动模拟器 + 控制器 + Web 可视化"""
    
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
        # 3. Web 可视化（浏览器访问 http://<IP>:5000）
        Node(
            package='warehouse_digital_twin',
            executable='web_visualizer',
            name='web_visualizer',
            output='screen'
        ),
    ])
