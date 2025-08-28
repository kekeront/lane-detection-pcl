#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2
import numpy as np
from geometry_msgs.msg import Point
import heapq
import math


def ros_to_numpy(msg):
    """Convert ROS PointCloud2 to numpy array [N,3]."""
    points = []
    for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])
    return np.array(points)


class RoadSquares(Node):
    def __init__(self):
        super().__init__("road_squares_node")

        # Subscribe to LiDAR topic
        self.create_subscription(
            PointCloud2,
            "/sensor/lidar_front/points",
            self.callback,
            10
        )

        # Publisher for semantic squares
        self.pub = self.create_publisher(Marker, "/semantic_map/squares", 10)

        self.square_size = 0.5
        self.half_size = self.square_size / 2.0

    def heuristic(self, a, b):
        """Euclidean distance heuristic."""
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def a_star(self, grid, start, goal):
        """Return shortest path from start to goal using A*."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            cx, cy = current
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                n = (cx+dx, cy+dy)
                if n not in grid:
                    continue
                tentative_g = g_score[current] + 1.0  # uniform step cost
                if n not in g_score or tentative_g < g_score[n]:
                    g_score[n] = tentative_g
                    f_score = tentative_g + self.heuristic(n, goal)
                    heapq.heappush(open_set, (f_score, n))
                    came_from[n] = current

        return []  # no path found

    def callback(self, msg: PointCloud2):
        pts = ros_to_numpy(msg)
        if pts.shape[0] == 0:
            return

        # Example filter: keep "road-like" points (replace with RANSAC segmentation later)
        road_pts = pts[pts[:,2] < 0.2]

        # Quantize into grid cells
        grid = set()
        for x, y, _ in road_pts:
            gx = int(x // self.square_size)
            gy = int(y // self.square_size)
            grid.add((gx, gy))

        if not grid:
            return

        # Start = closest grid cell to LiDAR (0,0)
        start = min(grid, key=lambda g: g[0]**2 + g[1]**2)

        # Goal = furthest cell in the grid (demo, you can change)
        goal = max(grid, key=lambda g: g[0]**2 + g[1]**2)

        # Run A*
        path = self.a_star(grid, start, goal)

        # Build visualization marker (show path only)
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "astar_node"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.1  # line thickness
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.9

        for gx, gy in path:
            p = Point()
            p.x = gx * self.square_size + self.half_size
            p.y = gy * self.square_size + self.half_size
            p.z = 0.0
            marker.points.append(p)

        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RoadSquares()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
