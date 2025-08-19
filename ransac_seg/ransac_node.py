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

        # Subscribe to LiDAR raw topic
        self.create_subscription(
            PointCloud2,
            "/sensor/lidar_front/points",
            self.callback,
            10
        )

        # Publish semantic squares (as visualization markers)
        self.pub = self.create_publisher(Marker, "/semantic_map/squares", 10)

    def callback(self, msg: PointCloud2):
        pts = ros_to_numpy(msg)
        if pts.shape[0] == 0:
            return

        # ✅ TODO: Apply your existing RANSAC road segmentation here
        # For now, I assume you already have road vs obstacle classification
        # Let's take a simple filter (ground plane / road_surface)
        road_pts = pts[pts[:,2] < 0.2]   # crude filter by z (replace with your RANSAC output)

        # Grid parameters
        square_size = 1.0   # size of each square in meters
        half_size = square_size / 2.0

        # Quantize road points into grid cells
        grid = set()
        for x, y, _ in road_pts:
            gx = int(x // square_size)
            gy = int(y // square_size)
            grid.add((gx, gy))

        # Publish one big marker with all squares
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "road_squares"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.scale.x = square_size
        marker.scale.y = square_size
        marker.scale.z = 0.05   # thin "tile"
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5    # transparency

        for gx, gy in grid:
            p = Point()
            p.x = gx * square_size + half_size
            p.y = gy * square_size + half_size
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
