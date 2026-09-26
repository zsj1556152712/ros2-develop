# -*- coding: utf-8 -*-
"""
导航任务：栅格地图上的 A* 寻路算法
- 实现经典 A*（开放列表 + 启发式，支持 8 方向移动）
- 搭建 15x15 栅格测试环境（含障碍物）
- 使用 OpenCV 可视化：网格、障碍、起点/终点、搜索范围、最终路径
"""
import heapq
import math
import os

import cv2
import numpy as np

# 输出目录固定为脚本所在目录，跨平台可移植
BASE = os.path.dirname(os.path.abspath(__file__))


class Node:
    """搜索节点：记录位置、代价与父节点"""

    __slots__ = ("x", "y", "g", "h", "f", "parent")

    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.g = 0.0          # 起点到当前节点的实际代价
        self.h = 0.0          # 当前节点到终点的启发式估计
        self.f = 0.0          # g + h
        self.parent = None    # 路径回溯指针

    def __lt__(self, other):
        return self.f < other.f


class AStar:
    """A* 寻路器：在 0/1 栅格上搜索最短路径（0=可通行，1=障碍）"""

    # 8 方向邻居偏移（dx, dy, 代价）
    NEIGHBORS8 = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
    ]

    def __init__(self, grid, start, goal):
        """
        :param grid: 二维 0/1 数组，1 表示障碍
        :param start: (x, y) 起点
        :param goal: (x, y) 终点
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.start = start
        self.goal = goal

    def _is_free(self, x: int, y: int) -> bool:
        """判断坐标是否在地图内且不是障碍"""
        return 0 <= x < self.cols and 0 <= y < self.rows and self.grid[y][x] == 0

    def _heuristic(self, x: int, y: int) -> float:
        """启发函数：欧氏距离（8 方向移动时一致且可采纳）"""
        gx, gy = self.goal
        return math.hypot(x - gx, y - gy)

    def search(self):
        """执行 A* 搜索，返回 (路径, 搜索过的节点集合)"""
        open_heap = []                      # 优先队列
        open_map = {}                       # (x,y) -> Node
        closed = {}                         # 已扩展节点

        start_node = Node(*self.start)
        start_node.h = self._heuristic(*self.start)
        start_node.f = start_node.h
        heapq.heappush(open_heap, start_node)
        open_map[self.start] = start_node

        while open_heap:
            current = heapq.heappop(open_heap)
            if (current.x, current.y) in closed:
                continue
            closed[(current.x, current.y)] = current

            # 到达终点：回溯路径
            if (current.x, current.y) == self.goal:
                path = []
                node = current
                while node is not None:
                    path.append((node.x, node.y))
                    node = node.parent
                return path[::-1], closed

            for dx, dy, cost in self.NEIGHBORS8:
                nx, ny = current.x + dx, current.y + dy
                if not self._is_free(nx, ny):
                    continue
                ng = current.g + cost
                key = (nx, ny)
                old = open_map.get(key)
                if old is None or ng < old.g:
                    node = Node(nx, ny)
                    node.g = ng
                    node.h = self._heuristic(nx, ny)
                    node.f = ng + node.h
                    node.parent = current
                    heapq.heappush(open_heap, node)
                    open_map[key] = node

        return None, closed  # 无可行路径


def build_grid():
    """构造 15x15 测试栅格：0 可通行，1 障碍（含一个不可穿越的"墙"）"""
    grid = [[0] * 15 for _ in range(15)]
    obstacles = [
        (7, 1), (7, 2), (7, 3), (7, 4), (7, 5), (7, 6), (7, 7), (7, 8),  # 竖直墙
        (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (7, 9), (8, 9), (9, 9),  # 横向墙
        (10, 11), (11, 11), (12, 11), (11, 12), (12, 12),                # 小块障碍
    ]
    for x, y in obstacles:
        grid[y][x] = 1
    return grid


def visualize(grid, start, goal, path, visited, out_path):
    """用 OpenCV 绘制 A* 搜索结果"""
    cell = 40
    h, w = len(grid), len(grid[0])
    info_bar = 40
    canvas = np.full((h * cell + info_bar, w * cell, 3), 255, np.uint8)

    # 障碍（深灰）
    for y in range(h):
        for x in range(w):
            if grid[y][x] == 1:
                cv2.rectangle(canvas, (x * cell, y * cell),
                              (x * cell + cell, y * cell + cell), (70, 70, 70), -1)
            elif (x, y) in visited:
                # 已搜索区域（浅蓝）
                cv2.rectangle(canvas, (x * cell, y * cell),
                              (x * cell + cell, y * cell + cell), (230, 240, 250), -1)

    # 网格线
    for i in range(w + 1):
        cv2.line(canvas, (i * cell, 0), (i * cell, h * cell), (200, 200, 200), 1)
    for j in range(h + 1):
        cv2.line(canvas, (0, j * cell), (w * cell, j * cell), (200, 200, 200), 1)

    # 最终路径（蓝色粗线）
    if path:
        pts = np.array([[x * cell + cell // 2, y * cell + cell // 2] for x, y in path], np.int32)
        cv2.polylines(canvas, [pts], False, (220, 80, 20), 6)
        for x, y in path:
            cv2.circle(canvas, (x * cell + cell // 2, y * cell + cell // 2), 5, (220, 80, 20), -1)

    # 起点（绿）与终点（红）
    sx, sy = start
    gx, gy = goal
    cv2.circle(canvas, (sx * cell + cell // 2, sy * cell + cell // 2), 12, (0, 180, 0), -1)
    cv2.circle(canvas, (gx * cell + cell // 2, gy * cell + cell // 2), 12, (0, 0, 220), -1)
    cv2.putText(canvas, "S", (sx * cell + 8, sy * cell + 27), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(canvas, "G", (gx * cell + 8, gy * cell + 27), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # 图例与统计
    info = "A* | path len: %d | visited: %d" % (len(path) if path else 0, len(visited))
    cv2.rectangle(canvas, (0, h * cell), (w * cell, h * cell + info_bar), (245, 245, 245), -1)
    cv2.putText(canvas, info, (8, h * cell + 27), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (30, 30, 30), 2)

    cv2.imwrite(out_path, canvas)
    print(info)


if __name__ == "__main__":
    grid = build_grid()
    start = (1, 1)
    goal = (13, 13)

    solver = AStar(grid, start, goal)
    path, visited = solver.search()

    if path:
        # 输出路径点序列与总代价
        total_cost = sum(math.hypot(x2 - x1, y2 - y1)
                         for (x1, y1), (x2, y2) in zip(path, path[1:]))
        print("路径点数:", len(path), "| 路径总代价: %.2f" % total_cost)
        print("路径:", " -> ".join("(%d,%d)" % p for p in path))
    else:
        print("未找到可行路径")

    visualize(grid, start, goal, path, visited, os.path.join(BASE, "astar_result.png"))
