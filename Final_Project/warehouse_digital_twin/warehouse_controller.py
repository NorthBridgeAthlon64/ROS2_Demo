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
        self.dt = 0.1  # 控制循环时间间隔（秒）
        self.timer = self.create_timer(self.dt, self.control_loop)
        
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
        """
        主控制循环，每0.1秒执行一次
        实现状态机控制逻辑：
        - IDLE: 等待任务开始
        - MOVING: 沿路径移动到目标点
        - ARRIVED: 到达目标点，执行任务
        - UNLOADING: 卸货操作
        - DONE: 任务完成
        """
        if self.state == 'IDLE':
            return  # 还没开始任务
        
        if self.state == 'DONE':
            return  # 任务已完成
        
        # 检查是否到达目标点（欧几里得距离）
        dist_to_target = math.sqrt((self.x - self.target_x)**2 + (self.y - self.target_y)**2)
        
        if dist_to_target < 0.3:  # 距离小于0.3米，算到达
            self.on_arrive()
        
        # 根据是否有规划路径选择移动方式
        if self.path_index < len(self.path):
            self.follow_path()  # 沿着规划路径移动
        else:
            self.move_to_target()  # 直接朝向目标移动
    
    def on_arrive(self):
        """
        到达目标点后的处理逻辑
        任务流程：
        - Step 1: 到达货架 → 抓取货物 → 规划到卸货区的路径
        - Step 2: 到达卸货区 → 卸货完成 → 任务结束
        """
        if self.task_step == 1:
            self.get_logger().info('✅ 到达货架！抓取货物...')
            self.publish_status('到达货架，抓取货物')
            self.task_step = 2
            
            # 第二个目标：去卸货区
            if len(self.drop_zones) > 0:
                # 计算卸货区的中心点坐标
                x_avg = sum(p[0] for p in self.drop_zones) / len(self.drop_zones)
                y_avg = sum(p[1] for p in self.drop_zones) / len(self.drop_zones)
                self.target_x = x_avg
                self.target_y = y_avg
                self.get_logger().info('前往卸货区: (%.2f, %.2f)' % (self.target_x, self.target_y))
                self.plan_path()  # 重新规划路径到卸货区
                self.publish_status('前往卸货区')
        
        elif self.task_step == 2:
            self.get_logger().info('✅ 到达卸货区！卸货完成！')
            self.publish_status('到达卸货区，卸货完成！')
            self.state = 'DONE'
            self.get_logger().info('🏆 任务全部完成！')
    
    def plan_path(self):
        """
        使用BFS算法规划从当前位置到目标位置的最短路径
        
        算法流程：
        1. 将世界坐标转换为网格坐标
        2. 使用广度优先搜索(BFS)寻找最短路径
        3. 将网格路径转换回世界坐标
        4. 存储路径点列表供跟踪控制使用
        
        返回：路径点列表（世界坐标），存储在self.path中
        """
        # 把当前位置和目标位置转成格子坐标
        start_row, start_col = world_to_grid(self.x, self.y)
        goal_row, goal_col = world_to_grid(self.target_x, self.target_y)
        
        # 使用BFS算法找最短路径
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
        广度优先搜索(BFS)算法寻找最短路径
        
        算法原理：
        - BFS从起点开始，逐层向外扩展搜索
        - 第一次到达目标点时就是最短路径
        - 使用队列存储待探索的节点
        - 使用visited集合避免重复访问
        
        参数：
            start_row, start_col: 起点网格坐标
            goal_row, goal_col: 目标点网格坐标
            
        返回：
            路径点列表 [(row, col), ...] 或 None（无路可走）
        """
        # 四个移动方向：上、下、左、右
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        # 队列存储：(row, col, path)
        # path是到达当前点的路径历史
        queue = deque()
        queue.append((start_row, start_col, []))
        visited = set()
        visited.add((start_row, start_col))
        
        while queue:
            row, col, path = queue.popleft()
            
            # 到达目标点，返回完整路径
            if row == goal_row and col == goal_col:
                return path + [(row, col)]
            
            # 探索四个方向的邻居节点
            for dr, dc in directions:
                new_row = row + dr
                new_col = col + dc
                
                # 检查边界和访问状态
                if (new_row, new_col) not in visited:
                    if get_cell(new_row, new_col) != 1:  # 不是墙壁（可通行）
                        visited.add((new_row, new_col))
                        queue.append((new_row, new_col, path + [(row, col)]))
        
        return None  # 无路可走
    
    def follow_path(self):
        """
        沿着规划好的路径移动（路径跟踪控制）
        
        控制策略：
        1. 获取当前路径点作为临时目标
        2. 如果到达当前路径点，切换到下一个
        3. 调用 move_towards() 朝路径点移动（带避障）
        """
        if self.path_index >= len(self.path):
            return
        
        # 获取当前路径点坐标
        waypoint_x, waypoint_y = self.path[self.path_index]
        
        # 距离这个路点多远
        dist = math.sqrt((self.x - waypoint_x)**2 + (self.y - waypoint_y)**2)
        
        # 到达当前路径点，切换到下一个
        if dist < 0.2:
            self.path_index += 1
            if self.path_index >= len(self.path):
                return  # 路径走完了
            waypoint_x, waypoint_y = self.path[self.path_index]
        
        # 朝当前路点移动
        self.move_towards(waypoint_x, waypoint_y)
    
    def move_to_target(self):
        """
        直接朝向目标移动（当路径规划失败时使用）
        调用 move_towards() 朝最终目标移动
        """
        self.move_towards(self.target_x, self.target_y)
    
    def move_towards(self, target_x, target_y):
        """
        朝目标点移动的简单控制算法（带激光避障）
        
        控制逻辑：
        1. 计算到目标点的距离和角度差
        2. 如果角度差很大，先原地转向
        3. 角度对准后，边前进边微调方向
        4. 使用激光雷达数据避障：前方有障碍物时减速并转弯
        
        参数：
            target_x, target_y: 目标点的世界坐标
        """
        # 计算到目标点的距离和角度
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        
        if dist < 0.1:  # 已经很接近目标，不需要移动
            return
        
        # 计算目标朝向角度
        target_theta = math.atan2(dy, dx)
        angle_diff = target_theta - self.theta
        # 角度归一化到 [-π, π] 范围
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        cmd = Twist()
        
        # 如果角度差很大，先原地转向
        if abs(angle_diff) > 0.2:
            cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
            cmd.linear.x = 0.0
        else:
            # 角度对准了，边前进边微调方向
            cmd.angular.z = angle_diff * 0.5  # 角度比例控制
            cmd.linear.x = min(0.3, dist / self.dt)  # 线速度限制
            
            # 激光雷达避障：检测正前方是否有障碍物
            if self.laser_ranges:
                front_distance = self.laser_ranges[0]  # 正前方距离（第0束激光）
                # 前方0.5米内有障碍物，停止前进并转弯
                if front_distance < 0.5:
                    cmd.linear.x = 0.0  # 停止前进
                    cmd.angular.z = 0.5  # 原地右转避障
                    self.get_logger().warn('前方障碍物！距离: %.2fm，转弯避障' % front_distance)
        
        self.cmd_pub.publish(cmd)
    
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
