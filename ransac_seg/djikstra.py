#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2
import numpy as np
from geometry_msgs.msg import Point
import heapq


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

    def dijkstra_connected(self, grid, root):
        """Return connected set of squares starting from root using Dijkstra."""
        visited = set()
        dist = {root: 0.0}
        pq = [(0.0, root)]  # (cost, node)

        while pq:
            cost, (cx, cy) = heapq.heappop(pq)
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))

            # 4-neighbors
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                n = (cx+dx, cy+dy)
                if n in grid and n not in visited:
                    new_cost = cost + 1.0  # uniform weight (grid step = 1)
                    if n not in dist or new_cost < dist[n]:
                        dist[n] = new_cost
                        heapq.heappush(pq, (new_cost, n))

        return visited

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

        # Root = closest grid cell to LiDAR (0,0)
        root = min(grid, key=lambda g: g[0]**2 + g[1]**2)

        # Dijkstra filter
        connected = self.dijkstra_connected(grid, root)

        # Build visualization marker
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "djikstra_squares"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.scale.x = self.square_size
        marker.scale.y = self.square_size
        marker.scale.z = 0.05
        marker.color.r = 0.2
        marker.color.g = 0.9   # brighter green to differentiate from DFS
        marker.color.b = 0.4
        marker.color.a = 0.7

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
