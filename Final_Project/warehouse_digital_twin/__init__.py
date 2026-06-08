"""
智能仓储机器人数字孪生系统

这是一个基于ROS2的仓储机器人仿真系统，包含：
- 机器人物理模拟
- 路径规划和导航
- 任务调度和控制
- 实时可视化界面
"""

__version__ = '1.0.0'
__author__ = 'student'
__email__ = 'student@example.com'

from .warehouse_map import (
    MAP_ROWS,
    MAP_COLS, 
    CELL_SIZE,
    WAREHOUSE_MAP,
    grid_to_world,
    world_to_grid,
    get_cell,
    find_cells_by_type
)

__all__ = [
    'MAP_ROWS',
    'MAP_COLS',
    'CELL_SIZE',
    'WAREHOUSE_MAP',
    'grid_to_world',
    'world_to_grid',
    'get_cell',
    'find_cells_by_type',
]