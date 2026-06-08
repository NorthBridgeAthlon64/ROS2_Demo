#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人物理模拟器
模拟机器人的运动、激光雷达、里程计
不依赖 Gazebo，纯数学计算
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry           # 里程计消息
from sensor_msgs.msg import LaserScan        # 激光雷达消息
from geometry_msgs.msg import Twist          # 速度指令
import math
from warehouse_digital_twin.warehouse_map import (
    MAP_ROWS, MAP_COLS, CELL_SIZE, WAREHOUSE_MAP,
    world_to_grid, get_cell
)


class RobotSimulator(Node):
    """
    机器人模拟器节点
    订阅 /cmd_vel（控制指令），发布 /odom（里程计）和 /scan（激光雷达）
    """
    def __init__(self):
        super().__init__('robot_simulator')
        
        # ====== 机器人状态 ======
        # 初始位置：地图左下角附近（1.0, 1.0）
        self.x = 1.0          # 世界坐标 X（米）
        self.y = 1.0          # 世界坐标 Y（米）
        self.theta = 0.0      # 朝向（弧度），0=朝右，π/2=朝上
        
        # 机器人物理参数
        self.wheel_base = 0.3  # 两轮间距（米），影响转弯半径
        
        # ====== 创建发布者 ======
        # 里程计：告诉别人机器人在哪、走多快
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # 激光雷达：模拟扫描周围的障碍物
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # ====== 创建订阅者 ======
        # 订阅控制指令，别人发 /cmd_vel 来控制机器人
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )
        
        # 当前速度
        self.vx = 0.0    # 线速度（m/s）
        self.vw = 0.0    # 角速度（rad/s）
        
        # ====== 定时器 ======
        # 每 0.1 秒更新一次（10Hz）
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.update)
        
        self.get_logger().info('机器人模拟器已启动，初始位置: (%.1f, %.1f)' % (self.x, self.y))
    
    def cmd_callback(self, msg):
        """
        收到控制指令时调用
        线速度（前进后退）
        角速度（左右转）
        """
        # ====== TODO 1: 从 Twist 消息中提取线速度和角速度 ======
        self.vx = msg.linear.x
        self.vw = msg.angular.z
    
    def update(self):
        """
        定时更新函数：模拟机器人运动，发布里程计和激光雷达数据
        
        这是物理模拟的核心，每0.1秒执行一次：
        1. 根据速度指令更新机器人位置（运动学模型）
        2. 检测碰撞并处理
        3. 发布里程计消息（位置、朝向、速度）
        4. 发布模拟的激光雷达扫描数据
        """
        # ====== 1. 更新位置（差速驱动运动学模型） ======
        # 差速模型假设：
        # - 机器人有两个驱动轮，左右轮速度可以独立控制
        # - 线速度 vx = (v_left + v_right) / 2
        # - 角速度 vw = (v_right - v_left) / wheel_base
        # - 位置更新：x += vx * cos(θ) * dt, y += vx * sin(θ) * dt
        # - 朝向更新：θ += vw * dt
        self.x += self.vx * math.cos(self.theta) * self.dt
        self.y += self.vx * math.sin(self.theta) * self.dt
        self.theta += self.vw * self.dt
        
        # 角度归一化到 [-π, π] 范围，避免数值溢出
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # ====== 2. 碰撞检测 ======
        # 检查机器人当前位置是否在障碍物（墙壁）内
        row, col = world_to_grid(self.x, self.y)
        if get_cell(row, col) == 1:  # 撞到障碍物
            # 简单的碰撞响应：回退到上一个位置
            self.x -= self.vx * math.cos(self.theta) * self.dt
            self.y -= self.vx * math.sin(self.theta) * self.dt
            self.vx = 0.0  # 停止运动
            self.vw = 0.0
            self.get_logger().warn('撞墙了！位置: (%.2f, %.2f)' % (self.x, self.y))
        
        # ====== 3. 发布里程计消息 ======
        self.publish_odometry()
        
        # ====== 4. 发布模拟的激光雷达数据 ======
        self.publish_laser_scan()
    
    def publish_odometry(self):
        """
        发布里程计消息（Odometry）
        
        里程计消息包含：
        - header: 时间戳和坐标系信息
        - pose.pose: 机器人位置（x,y）和朝向（四元数）
        - twist.twist: 机器人当前速度（线速度和角速度）
        
        坐标系说明：
        - frame_id: 'odom' - 固定世界坐标系
        - child_frame_id: 'base_link' - 机器人本体坐标系
        """
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'          # 里程计坐标系
        msg.child_frame_id = 'base_link'      # 机器人坐标系
        
        # 位置信息
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        
        # 朝向信息（使用四元数表示旋转）
        # 绕Z轴旋转theta弧度，四元数公式：
        # qz = sin(θ/2), qw = cos(θ/2)
        msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        # 速度信息
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.vw
        
        self.odom_pub.publish(msg)
    
    def publish_laser_scan(self):
        """
        发布模拟的激光雷达扫描数据（LaserScan）
        
        激光雷达参数：
        - 360°全方位扫描，每1°一个测距点
        - 最小检测距离0.1米，最大检测距离5.0米
        - 使用光线投射算法模拟真实激光雷达
        
        算法原理：
        对每个角度，从机器人位置发射射线，逐步检测是否碰到障碍物
        """
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        
        # 激光雷达配置参数
        msg.angle_min = 0.0                     # 起始角度（弧度）
        msg.angle_max = 2 * math.pi             # 结束角度（360度）
        msg.angle_increment = math.pi / 180     # 角度间隔（1度）
        msg.range_min = 0.1                     # 最小检测距离（米）
        msg.range_max = 5.0                     # 最大检测距离（米）
        
        # 计算每个方向的距离
        ranges = []
        num_beams = 360  # 360条射线
        
        for i in range(num_beams):
            angle = msg.angle_min + i * msg.angle_increment
            # 转换成世界坐标系下的角度（机器人朝向 + 激光雷达相对角度）
            world_angle = self.theta + angle
            distance = self.ray_cast(world_angle)
            ranges.append(distance)
        
        msg.ranges = ranges
        self.scan_pub.publish(msg)
    
    def ray_cast(self, angle):
        """
        光线投射算法：模拟单条激光射线的测距过程
        
        算法步骤：
        1. 从机器人当前位置出发
        2. 沿指定角度方向逐步前进（步长0.1米）
        3. 每前进一步，检查该点是否在障碍物内
        4. 如果碰到障碍物，返回当前距离
        5. 如果达到最大距离仍未碰到，返回最大距离
        
        参数：
            angle: 射线方向（弧度，世界坐标系）
            
        返回：
            最近障碍物的距离（米），如果没找到返回5.0米
        """
        max_range = 5.0
        step = 0.1  # 每步检测间隔（米），精度和性能的平衡
        distance = 0.0
        
        while distance < max_range:
            # 计算射线终点的世界坐标
            ray_x = self.x + distance * math.cos(angle)
            ray_y = self.y + distance * math.sin(angle)
            
            # 转换成地图格子坐标
            row, col = world_to_grid(ray_x, ray_y)
            
            # 检查这个格子是否是障碍物（墙壁）
            if get_cell(row, col) == 1:
                return distance  # 找到障碍物，返回距离
            
            distance += step
        
        return max_range  # 没找到障碍物，返回最大距离


def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
