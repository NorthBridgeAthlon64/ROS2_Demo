#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仓储机器人可视化界面
用 ASCII 画地图、机器人和轨迹
在终端中实时显示系统状态
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import os
from warehouse_digital_twin.warehouse_map import (
    MAP_ROWS, MAP_COLS, CELL_SIZE, WAREHOUSE_MAP,
    world_to_grid
)


class Visualizer(Node):
    """
    可视化节点
    订阅 /odom（里程计）和 /task_status（任务状态）
    在终端中画出地图和机器人位置
    """

    # 用于显示的字符映射
    CELL_CHARS = {
        0: '  ',  # 空地
        1: '##',  # 墙壁
        2: '[]',  # 货架
        3: '<>',  # 卸货区
    }

    def __init__(self):
        super().__init__('visualizer')

        # 机器人位置
        self.x = 1.0
        self.y = 1.0
        self.theta = 0.0

        # 任务状态
        self.status = '等待中...'

        # 轨迹记录（存储走过的世界坐标点）
        self.trajectory = []
        self.traj_counter = 0  # 计数，隔几帧记录一个点

        # 帧计数
        self.frame_count = 0

        # ====== 订阅 ======
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/task_status', self.status_callback, 10
        )

        # ====== 定时刷新（5Hz） ======
        self.timer = self.create_timer(0.2, self.draw)

        self.get_logger().info('可视化界面已启动')

    def odom_callback(self, msg):
        """收到里程计数据，更新机器人位置和轨迹"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # 从四元数提取朝向
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.theta = 2 * math.atan2(z, w)

        # 记录轨迹（每5帧记录一个点）
        self.traj_counter += 1
        if self.traj_counter % 5 == 0:
            self.trajectory.append((self.x, self.y))
            if len(self.trajectory) > 300:  # 限制轨迹点数量
                self.trajectory.pop(0)

    def status_callback(self, msg):
        """收到任务状态"""
        self.status = msg.data

    def draw(self):
        """绘制可视化界面"""
        # 清屏
        os.system('cls' if os.name == 'nt' else 'clear')

        self.frame_count += 1

        # ====== 标题 ======
        print('=' * 60)
        print('             智能仓储机器人数字孪生系统')
        print('=' * 60)

        # 获取机器人在格子中的位置
        robot_row, robot_col = world_to_grid(self.x, self.y)

        # ====== 收集轨迹格子坐标（用于查表） ======
        traj_grid = set()
        for tx, ty in self.trajectory:
            tr, tc = world_to_grid(tx, ty)
            traj_grid.add((tr, tc))

        # ====== 画地图 ======
        for row in range(MAP_ROWS):
            line = ''
            for col in range(MAP_COLS):
                if row == robot_row and col == robot_col:
                    # 用箭头表示机器人朝向
                    angle_deg = math.degrees(self.theta)
                    if -45 <= angle_deg < 45:
                        line += '► '  # 朝右
                    elif 45 <= angle_deg < 135:
                        line += '▲ '  # 朝上
                    elif 135 <= angle_deg < 180 or -180 <= angle_deg < -135:
                        line += '◄ '  # 朝左
                    else:
                        line += '▼ '  # 朝下
                elif (row, col) in traj_grid:
                    line += '· '  # 轨迹点
                else:
                    cell_type = WAREHOUSE_MAP[row][col]
                    line += self.CELL_CHARS[cell_type]
            print(line)

        # ====== 状态信息面板 ======
        print('=' * 60)
        print('  [任务状态]: %s' % self.status)
        print('  [位置]: X = %.2f m, Y = %.2f m' % (self.x, self.y))
        print('  [朝向]: %.1f°' % math.degrees(self.theta))
        print('  [轨迹点]: %d 个' % len(self.trajectory))
        print('=' * 60)

        # ====== 图例 ======
        print('  图例: ## 墙壁  [] 货架  <> 卸货区  ►▲◄▼ 机器人  · 轨迹')
        print('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
