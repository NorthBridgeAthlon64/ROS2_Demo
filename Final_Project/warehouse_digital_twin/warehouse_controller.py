#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仓储机器人控制器
负责任务调度和导航控制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
from warehouse_digital_twin.warehouse_map import (
    MAP_ROWS, MAP_COLS, CELL_SIZE, WAREHOUSE_MAP,
    find_cells_by_type, world_to_grid, grid_to_world, get_cell
)
from collections import deque


class WarehouseController(Node):
    """
    仓储控制器节点
    订阅 /odom 和 /scan，发布 /cmd_vel
    实现任务调度和简单的导航
    """
    def __init__(self):
        super().__init__('warehouse_controller')
        
        # ====== 发布者 ======
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        
        # ====== 订阅者 ======
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # ====== 机器人状态 ======
        self.x = 1.0
        self.y = 1.0
        self.theta = 0.0
        self.laser_ranges = []  # 激光雷达数据
        
        # ====== 任务状态 ======
        # 状态机：IDLE→MOVING→ARRIVED→MOVING→UNLOADING→DONE
        self.state = 'IDLE'
        self.target_x = 0.0
        self.target_y = 0.0
        self.task_step = 0  # 任务步骤
        
        # 找到地图上的关键位置
        self.shelves = find_cells_by_type(2)  # 货架位置列表
        self.drop_zones = find_cells_by_type(3)  # 卸货区位置列表
        self.start_pos = (1.0, 1.0)  # 起始位置
        
        # ====== 路径规划相关 ======
        self.path = []  # 当前规划好的路径
        self.path_index = 0  # 当前走到路径的第几个点
        
        # ====== 定时器 ======
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('仓储控制器已启动')
        self.get_logger().info('货架位置: %s' % str(self.shelves))
        self.get_logger().info('卸货区位置: %s' % str(self.drop_zones))
        
        # 延迟启动任务
        self.start_delay = self.create_timer(2.0, self.start_task)
    
    def start_task(self):
        """延迟2秒后开始任务"""
        self.start_delay.cancel()  # 取消这个定时器
        self.state = 'MOVING'
        self.task_step = 1
        
        # 第一个目标：去第一个货架
        if len(self.shelves) > 0:
            self.target_x, self.target_y = self.shelves[0]
            self.get_logger().info('任务开始！前往货架: (%.2f, %.2f)' % (self.target_x, self.target_y))
            self.plan_path()  # 规划路径
            self.publish_status('任务开始：前往货架')
    
    def odom_callback(self, msg):
        """收到里程计数据"""
        # ====== TODO 4: 从里程计数据中提取机器人位置 ======
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # 从四元数提取朝向
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.theta = 2 * math.atan2(z, w)
    
    def scan_callback(self, msg):
        """收到激光雷达数据"""
        self.laser_ranges = msg.ranges
    
    def control_loop(self):
        """主控制循环，每0.1秒执行一次"""
        if self.state == 'IDLE':
            return  # 还没开始任务
        
        if self.state == 'DONE':
            return  # 任务已完成
        
        # 检查是否到达目标
        dist_to_target = math.sqrt((self.x - self.target_x)**2 + (self.y - self.target_y)**2)
        
        if dist_to_target < 0.3:  # 距离小于0.3米，算到达
            self.on_arrive()
        
        # 沿着路径走
        if self.path_index < len(self.path):
            self.follow_path()
        else:
            # 路径走完了，直接朝向目标
            self.move_to_target()
    
    def on_arrive(self):
        """到达目标点后的处理"""
        if self.task_step == 1:
            self.get_logger().info('✅ 到达货架！抓取货物...')
            self.publish_status('到达货架，抓取货物')
            self.task_step = 2
            
            # 第二个目标：去卸货区
            if len(self.drop_zones) > 0:
                # 找卸货区的中心点
                x_avg = sum(p[0] for p in self.drop_zones) / len(self.drop_zones)
                y_avg = sum(p[1] for p in self.drop_zones) / len(self.drop_zones)
                self.target_x = x_avg
                self.target_y = y_avg
                self.get_logger().info('前往卸货区: (%.2f, %.2f)' % (self.target_x, self.target_y))
                self.plan_path()
                self.publish_status('前往卸货区')
        
        elif self.task_step == 2:
            self.get_logger().info('✅ 到达卸货区！卸货完成！')
            self.publish_status('到达卸货区，卸货完成！')
            self.state = 'DONE'
            self.get_logger().info('🏆 任务全部完成！')
    
    def plan_path(self):
        """
        用 BFS 规划从当前位置到目标位置的最短路径
        返回路径点列表（世界坐标）
        """
        # 把当前位置和目标位置转成格子坐标
        start_row, start_col = world_to_grid(self.x, self.y)
        goal_row, goal_col = world_to_grid(self.target_x, self.target_y)
        
        # BFS 找最短路径
        path = self.bfs(start_row, start_col, goal_row, goal_col)
        
        if path:
            # 把格子路径转成世界坐标路径
            self.path = [grid_to_world(r, c) for r, c in path]
            self.path_index = 0
            self.get_logger().info('路径规划完成，共 %d 个路点' % len(self.path))
        else:
            self.get_logger().warn('找不到路径！将直线前进')
            self.path = [(self.target_x, self.target_y)]
            self.path_index = 0
    
    def bfs(self, start_row, start_col, goal_row, goal_col):
        """
        BFS 广度优先搜索，找最短路径
        返回格子坐标列表 [(row, col), ...]
        """
        # 四个方向：上、下、左、右
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        # 队列存储：(row, col, path)
        queue = deque()
        queue.append((start_row, start_col, []))
        visited = set()
        visited.add((start_row, start_col))
        
        while queue:
            row, col, path = queue.popleft()
            
            # 到达目标
            if row == goal_row and col == goal_col:
                return path + [(row, col)]
            
            # 探索四个方向
            for dr, dc in directions:
                new_row = row + dr
                new_col = col + dc
                
                # 检查是否在地图范围内且未访问过
                if (new_row, new_col) not in visited:
                    if get_cell(new_row, new_col) != 1:  # 不是墙壁
                        visited.add((new_row, new_col))
                        queue.append((new_row, new_col, path + [(row, col)]))
        
        return None  # 无路可走
    
    def follow_path(self):
        """沿着规划好的路径移动"""
        if self.path_index >= len(self.path):
            return
        
        target_x, target_y = self.path[self.path_index]
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        
        if dist < 0.2:  # 到达当前路径点
            self.path_index += 1
            return
        
        # 计算目标角度
        target_theta = math.atan2(dy, dx)
        angle_diff = target_theta - self.theta
        # 归一化到 [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 发布速度指令
        msg = Twist()
        if abs(angle_diff) > 0.3:  # 先转弯再走
            msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            msg.linear.x = 0.0
        else:
            msg.linear.x = min(0.3, dist / self.dt)
            msg.angular.z = 0.5 * angle_diff
        
        self.cmd_pub.publish(msg)
    
    def move_to_target(self):
        """直接朝向目标移动（没有路径时使用）"""
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        
        if dist < 0.1:
            return
        
        # 计算目标角度
        target_theta = math.atan2(dy, dx)
        angle_diff = target_theta - self.theta
        # 归一化到 [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        msg = Twist()
        if abs(angle_diff) > 0.2:  # 先转弯
            msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            msg.linear.x = 0.0
        else:
            msg.linear.x = min(0.3, dist / self.dt)
            msg.angular.z = 0.5 * angle_diff
        
        self.cmd_pub.publish(msg)
    
    def publish_status(self, status_text):
        """发布任务状态消息"""
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
