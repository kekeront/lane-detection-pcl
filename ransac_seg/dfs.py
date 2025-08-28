#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2
import numpy as np
from geometry_msgs.msg import Point


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

    def dfs_connected(self, grid, root):
        """Return connected set of squares starting from root using DFS."""
        visited = set()
        stack = [root]
        visited.add(root)

        while stack:
            cx, cy = stack.pop()
            # 4-neighbors (use 8 for diagonals if needed)
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                n = (cx+dx, cy+dy)
                if n in grid and n not in visited:
                    visited.add(n)
                    stack.append(n)
        return visited

    def callback(self, msg: PointCloud2):
        pts = ros_to_numpy(msg)
        if pts.shape[0] == 0:
            return

        # Example filter: keep "road-like" points (replace with RANSAC segmentation)
        road_pts = pts[pts[:,2] < 0.2]

        # Quantize into grid cells
        grid = set()
        for x, y, _ in road_pts:
            gx = int(x // self.square_size)
            gy = int(y // self.square_size)
            grid.add((gx, gy))

        if not grid:
            return

        # Root = closest grid cell to LiDAR (0,0)
        root = min(grid, key=lambda g: g[0]**2 + g[1]**2)

        # DFS filter
        connected = self.dfs_connected(grid, root)

        # Build visualization marker
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dfs_squares"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.scale.x = self.square_size
        marker.scale.y = self.square_size
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.5   # slightly darker green than BFS
        marker.color.b = 1.0
        marker.color.a = 0.6

        for gx, gy in connected:
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
