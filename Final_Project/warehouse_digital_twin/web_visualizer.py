#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Web 可视化界面
通过 Flask 提供浏览器端的实时仓库地图显示
支持 SSH 远程访问，在浏览器中查看机器人运行
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import json
import threading
import time
from warehouse_digital_twin.warehouse_map import (
    MAP_ROWS, MAP_COLS, CELL_SIZE, WAREHOUSE_MAP,
    world_to_grid,
)


# ==================== Flask Web 服务器 ====================

HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>智能仓储机器人数字孪生系统</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body {
    font-family: 'Consolas', 'Courier New', monospace;
    background: #1a1a2e;
    color: #e0e0e0;
    display: flex;
    flex-direction: column;
    align-items: center;
    min-height: 100vh;
    padding: 20px;
}
h1 {
    color: #00d2ff;
    margin-bottom: 5px;
    font-size: 24px;
}
.subtitle {
    color: #888;
    font-size: 13px;
    margin-bottom: 15px;
}
.panel {
    background: #16213e;
    border: 2px solid #0f3460;
    border-radius: 8px;
    padding: 15px;
    margin-bottom: 15px;
    text-align: center;
}
#robot-info {
    display: flex;
    gap: 25px;
    justify-content: center;
    flex-wrap: wrap;
}
.info-item {
    display: flex;
    flex-direction: column;
    align-items: center;
}
.info-label {
    font-size: 12px;
    color: #888;
    text-transform: uppercase;
}
.info-value {
    font-size: 20px;
    font-weight: bold;
    color: #00d2ff;
}
#status {
    font-size: 16px;
    font-weight: bold;
    padding: 5px 15px;
    border-radius: 5px;
}
.status-moving { color: #ffd700; }
.status-done { color: #00ff88; }
.status-pickup { color: #ff6ec7; }
canvas {
    border: 2px solid #0f3460;
    border-radius: 4px;
    background: #0a0a1a;
}
.legend {
    display: flex;
    gap: 20px;
    margin-top: 10px;
    font-size: 13px;
    color: #aaa;
    flex-wrap: wrap;
    justify-content: center;
}
.legend span { display: flex; align-items: center; gap: 5px; }
.legend-box {
    display: inline-block;
    width: 16px; height: 16px;
    border-radius: 2px;
}
</style>
</head>
<body>
<h1>&#x1F3ED; 智能仓储机器人数字孪生系统</h1>
<div class="subtitle">纯代码实现 &middot; 浏览器实时监控 &middot; SSH 友好</div>

<div class="panel" id="robot-info">
    <div class="info-item">
        <span class="info-label">&#x1F4CD; X 坐标</span>
        <span class="info-value" id="info-x">--</span>
    </div>
    <div class="info-item">
        <span class="info-label">&#x1F4CD; Y 坐标</span>
        <span class="info-value" id="info-y">--</span>
    </div>
    <div class="info-item">
        <span class="info-label">&#x1F9ED; 朝向</span>
        <span class="info-value" id="info-theta">--</span>
    </div>
    <div class="info-item">
        <span class="info-label">&#x1F4CB; 任务状态</span>
        <span id="status">等待中...</span>
    </div>
</div>

<canvas id="map" width="640" height="640"></canvas>

<div class="legend">
    <span><span class="legend-box" style="background:#444"></span> 墙壁</span>
    <span><span class="legend-box" style="background:#c07830"></span> 货架</span>
    <span><span class="legend-box" style="background:#2a7fff"></span> 卸货区</span>
    <span>&#x1F916; 机器人</span>
    <span><span style="color:#ff0">&#x25CF;</span> 轨迹</span>
</div>

<script>
const CELL = 32;  // 每个格子的像素大小
const MAP_ROWS = MAP_ROWS_PLACEHOLDER;
const MAP_COLS = MAP_COLS_PLACEHOLDER;
const CELL_SIZE = CELL_SIZE_PLACEHOLDER;  // 米/格
const WAREHOUSE_MAP = WAREHOUSE_MAP_PLACEHOLDER;

const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');

let robotX = 1.0, robotY = 1.0, robotTheta = 0.0;
let trajectory = [];
let taskStatus = '等待中...';

// 预定义颜色
const COLORS = {
    0: '#0a0a1a',  // 空地
    1: '#3a3a5c',  // 墙壁
    2: '#c07830',  // 货架
    3: '#1a4a8a',  // 卸货区
};

function drawMap() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 绘制地图格子
    for (let row = 0; row < MAP_ROWS; row++) {
        for (let col = 0; col < MAP_COLS; col++) {
            let cellType = WAREHOUSE_MAP[row][col];
            ctx.fillStyle = COLORS[cellType] || '#0a0a1a';
            ctx.fillRect(col * CELL, row * CELL, CELL, CELL);
            // 格子线
            ctx.strokeStyle = '#111';
            ctx.lineWidth = 0.5;
            ctx.strokeRect(col * CELL, row * CELL, CELL, CELL);
        }
    }

    // 绘制轨迹
    for (let i = 0; i < trajectory.length; i++) {
        let p = trajectory[i];
        let col = Math.floor(p.x / CELL_SIZE);
        let row = MAP_ROWS - 1 - Math.floor(p.y / CELL_SIZE);
        let cx = col * CELL + CELL / 2;
        let cy = row * CELL + CELL / 2;
        ctx.fillStyle = 'rgba(255, 255, 0, 0.5)';
        ctx.beginPath();
        ctx.arc(cx, cy, 3, 0, Math.PI * 2);
        ctx.fill();
    }

    // 绘制机器人
    let col = Math.floor(robotX / CELL_SIZE);
    let row = MAP_ROWS - 1 - Math.floor(robotY / CELL_SIZE);
    let cx = col * CELL + CELL / 2;
    let cy = row * CELL + CELL / 2;

    // 机器人底盘
    ctx.fillStyle = '#00ff88';
    ctx.beginPath();
    ctx.arc(cx, cy, CELL * 0.35, 0, Math.PI * 2);
    ctx.fill();

    // 方向指示线
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(
        cx + Math.cos(robotTheta) * CELL * 0.4,
        cy - Math.sin(robotTheta) * CELL * 0.4
    );
    ctx.stroke();

    // 机器人中心点
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fill();
}

function updateInfo(data) {
    robotX = data.x;
    robotY = data.y;
    robotTheta = data.theta;
    taskStatus = data.status || '等待中...';
    trajectory = data.trajectory || [];

    document.getElementById('info-x').textContent = robotX.toFixed(2) + ' m';
    document.getElementById('info-y').textContent = robotY.toFixed(2) + ' m';
    document.getElementById('info-theta').textContent = (robotTheta * 180 / Math.PI).toFixed(1) + '°';

    let statusEl = document.getElementById('status');
    statusEl.textContent = taskStatus;
    statusEl.className = '';
    if (taskStatus.includes('完成')) statusEl.className = 'status-done';
    else if (taskStatus.includes('货架')) statusEl.className = 'status-pickup';
    else if (taskStatus.includes('前往')) statusEl.className = 'status-moving';

    drawMap();
}

// SSE: 接收服务器推送的实时数据
const evtSource = new EventSource('/stream');
evtSource.onmessage = function(event) {
    let data = JSON.parse(event.data);
    updateInfo(data);
};
evtSource.onerror = function() {
    console.log('SSE 连接中断，正在重连...');
};
</script>
</body>
</html>"""


class WebVisualizer(Node):
    """
    Web 可视化节点
    订阅 /odom 和 /task_status，通过 Flask 提供浏览器可视化
    """
    def __init__(self):
        super().__init__('web_visualizer')

        # ====== 机器人状态（线程安全） ======
        self._lock = threading.Lock()
        self.robot_x = 1.0
        self.robot_y = 1.0
        self.robot_theta = 0.0
        self.task_status = '等待任务开始...'
        self.trajectory = []  # [(x, y), ...]
        self.traj_counter = 0

        # ====== 订阅者 ======
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(String, '/task_status', self.status_callback, 10)

        # ====== 启动 Flask 服务器 ======
        self.app = self._create_flask_app()
        self._server_thread = threading.Thread(target=self._run_flask, daemon=True)
        self._server_thread.start()

        self.get_logger().info('Web 可视化已启动: http://0.0.0.0:5000')

    def odom_callback(self, msg):
        """收到里程计数据"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        theta = 2 * math.atan2(z, w)

        with self._lock:
            self.robot_x = x
            self.robot_y = y
            self.robot_theta = theta

            # 记录轨迹
            self.traj_counter += 1
            if self.traj_counter % 5 == 0:
                self.trajectory.append((x, y))
                if len(self.trajectory) > 500:
                    self.trajectory = self.trajectory[-500:]

    def status_callback(self, msg):
        """收到任务状态"""
        with self._lock:
            self.task_status = msg.data

    def get_state(self):
        """获取当前机器人状态（线程安全）"""
        with self._lock:
            return {
                'x': self.robot_x,
                'y': self.robot_y,
                'theta': self.robot_theta,
                'status': self.task_status,
                'trajectory': list(self.trajectory),
            }

    # ==================== Flask ====================

    def _create_flask_app(self):
        """创建 Flask 应用"""
        try:
            from flask import Flask, Response, request
        except ImportError:
            self.get_logger().error('Flask 未安装！请运行: pip install flask')
            raise

        app = Flask(__name__)
        node_ref = self  # 闭包引用

        # 用实际地图数据替换 HTML 占位符
        map_json = json.dumps(WAREHOUSE_MAP)
        rendered_html = HTML_PAGE.replace('MAP_ROWS_PLACEHOLDER', str(MAP_ROWS))
        rendered_html = rendered_html.replace('MAP_COLS_PLACEHOLDER', str(MAP_COLS))
        rendered_html = rendered_html.replace('CELL_SIZE_PLACEHOLDER', str(CELL_SIZE))
        rendered_html = rendered_html.replace('WAREHOUSE_MAP_PLACEHOLDER', map_json)

        @app.route('/')
        def index():
            return rendered_html, 200, {'Content-Type': 'text/html; charset=utf-8'}

        @app.route('/stream')
        def stream():
            """SSE 端点，持续推送机器人状态"""
            def generate():
                while True:
                    state = node_ref.get_state()
                    yield f'data: {json.dumps(state)}\n\n'
                    time.sleep(0.2)

            return Response(
                generate(),
                mimetype='text/event-stream',
                headers={
                    'Cache-Control': 'no-cache',
                    'Connection': 'keep-alive',
                    'X-Accel-Buffering': 'no',
                }
            )

        return app

    def _run_flask(self):
        """在后台线程运行 Flask"""
        try:
            from flask import Flask
        except ImportError:
            return
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


def main(args=None):
    rclpy.init(args=args)

    # 检查 Flask 是否安装
    try:
        import flask  # noqa: F401
    except ImportError:
        print('=' * 60)
        print('  [错误] Flask 未安装！')
        print('  请运行: pip install flask')
        print('=' * 60)
        return

    node = WebVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
