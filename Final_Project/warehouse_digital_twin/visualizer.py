#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仓储机器人可视化界面
用终端 ASCII 图形显示仓库地图和机器人位置
美观、实用、不需要任何图形库
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
    在终端中绘制 ASCII 地图、机器人和轨迹
    
    特点：
    - 使用线框字符画边框，美观大方
    - 实时显示机器人位置（🤖）和轨迹（○）
    - 地图元素用 emoji 区分：📦=货架, 🏁=卸货区, █=墙壁
    - 每0.5秒刷新一次
    """
    def __init__(self):
        super().__init__('visualizer')

        # ====== 机器人状态 ======
        self.robot_x = 1.0
        self.robot_y = 1.0
        self.robot_theta = 0.0

        # ====== 任务状态 ======
        self.task_status = '等待任务开始...'

        # ====== 轨迹记录 ======
        self.robot_trajectory = []  # 走过的世界坐标点
        self.traj_counter = 0  # 轨迹计数器（隔几帧记录一个点）

        # ====== 符号定义 ======
        # 使用 emoji 和特殊字符，美观直观
        self.SYMBOLS = {
            0: '· ',  # 空地
            1: '█',   # 墙壁
            2: '📦',  # 货架
            3: '🏁',  # 卸货区
        }

        # ====== 订阅者 ======
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/task_status', self.status_callback, 10
        )

        # ====== 定时器（2Hz刷新） ======
        self.timer = self.create_timer(0.5, self.redraw)

        self.get_logger().info('可视化界面已启动')
        self.get_logger().info('请在全屏终端中查看（建议终端窗口至少 80×40）')

    def odom_callback(self, msg):
        """
        收到里程计数据，更新机器人位置和轨迹
        
        从 /odom 消息中提取：
        - position.x, position.y: 机器人世界坐标
        - orientation.z, orientation.w: 四元数表示的朝向
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # 从四元数提取朝向角度（绕Z轴旋转）
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.robot_theta = 2 * math.atan2(z, w)

        # 记录轨迹（每3帧记录一个点，约0.5秒一个点）
        self.traj_counter += 1
        if self.traj_counter % 3 == 0:
            self.robot_trajectory.append((self.robot_x, self.robot_y))
            # 只保留最近200个轨迹点
            if len(self.robot_trajectory) > 200:
                self.robot_trajectory = self.robot_trajectory[-200:]

    def status_callback(self, msg):
        """收到任务状态更新"""
        self.task_status = msg.data

    def redraw(self):
        """
        重新绘制整个可视化界面
        
        布局：
        ╔═══ 标题栏 ═══╗
        ║  状态信息      ║
        ╠═══ 地图区域 ═══╣
        ║  20×20 地图    ║
        ╠═══ 图例说明 ═══╣
        ║  符号说明      ║
        ╚═══════════════╝
        """
        # 清屏（根据操作系统选择命令）
        os.system('cls' if os.name == 'nt' else 'clear')

        # ====== 标题栏 ======
        print('╔══════════════════════════════════════════════════════════════╗')
        print('║        智能仓储机器人数字孪生系统                            ║')
        print('║        纯代码实现 · 无仿真器 · 无 rviz2                      ║')
        print('╠══════════════════════════════════════════════════════════════╣')

        # ====== 任务状态 ======
        status_str = '║  任务状态: %-50s ║' % self.task_status[:50]
        print(status_str)

        # ====== 机器人位置信息 ======
        pos_str = '║  机器人位置: (%.2f, %.2f)  朝向: %.1f°%-20s ║' % (
            self.robot_x, self.robot_y,
            math.degrees(self.robot_theta),
            ''
        )
        print(pos_str)

        print('╠══════════════════════════════════════════════════════════════╣')

        # ====== 绘制地图 ======
        self.draw_map()

        # ====== 图例 ======
        print('╚══════════════════════════════════════════════════════════════╝')
        print('  图例:  · = 空地  █ = 墙壁  📦 = 货架  🏁 = 卸货区  🤖 = 机器人  ○ = 轨迹')

    def draw_map(self):
        """
        绘制仓库地图
        
        逐行扫描地图，在每个格子里绘制对应的符号：
        - 机器人所在格子 → 🤖
        - 轨迹点所在格子 → ○
        - 其他 → 地图定义中的符号
        """
        # 把机器人位置转成格子坐标
        robot_row, robot_col = world_to_grid(self.robot_x, self.robot_y)

        # 轨迹点也转成格子坐标（用集合加速查表）
        trajectory_cells = set()
        for tx, ty in self.robot_trajectory:
            tr, tc = world_to_grid(tx, ty)
            trajectory_cells.add((tr, tc))

        # 逐行绘制地图
        for row in range(MAP_ROWS):
            line = '  '  # 左侧留两个空格
            for col in range(MAP_COLS):
                # 判断这个格子里应该显示什么
                if row == robot_row and col == robot_col:
                    # 机器人在这里 → 显示机器人 emoji
                    line += self.get_robot_emoji()
                elif (row, col) in trajectory_cells:
                    # 走过这个格子 → 显示轨迹点
                    line += '○ '
                else:
                    # 正常的地图内容
                    cell_type = WAREHOUSE_MAP[row][col]
                    line += self.SYMBOLS.get(cell_type, '  ')

            # 给每行加上边框
            print('║' + line + ' ║')

    def get_robot_emoji(self):
        """
        根据机器人朝向返回对应的显示符号
        
        机器人始终用 🤖 表示，emoji 方便识别
        """
        return '🤖'


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
