#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2
import numpy as np
from geometry_msgs.msg import Point
from scipy.interpolate import UnivariateSpline


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

        # Publishers: squares + left/right curves
        self.pub_squares = self.create_publisher(Marker, "/semantic_map/squares", 10)
        self.pub_left = self.create_publisher(Marker, "/semantic_map/road_left", 10)
        self.pub_right = self.create_publisher(Marker, "/semantic_map/road_right", 10)

    def callback(self, msg: PointCloud2):
        pts = ros_to_numpy(msg)
        if pts.shape[0] == 0:
            return

        # ✅ TODO: Apply your existing RANSAC road segmentation here
        road_pts = pts[pts[:, 2] < 0.2]   # crude filter by z

        if len(road_pts) < 30:
            return

        # ------------------------
        # GRID SQUARES
        # ------------------------
        square_size = 0.5
        half_size = square_size / 2.0
        grid = set()
        for x, y, _ in road_pts:
            gx = int(x // square_size)
            gy = int(y // square_size)
            grid.add((gx, gy))

        squares = Marker()
        squares.header.frame_id = msg.header.frame_id
        squares.header.stamp = self.get_clock().now().to_msg()
        squares.ns = "road_squares"
        squares.id = 0
        squares.type = Marker.CUBE_LIST
        squares.action = Marker.ADD
        squares.scale.x = square_size
        squares.scale.y = square_size
        squares.scale.z = 0.05
        squares.color.r = 0.0
        squares.color.g = 1.0
        squares.color.b = 0.0
        squares.color.a = 0.5
        squares.lifetime = Duration(seconds=60).to_msg()

        for gx, gy in grid:
            p = Point()
            p.x = gx * square_size + half_size
            p.y = gy * square_size + half_size
            p.z = 0.0
            squares.points.append(p)

        self.pub_squares.publish(squares)

        # ------------------------
        # CURVE BOUNDARIES
        # ------------------------
        x = road_pts[:, 0]
        y = road_pts[:, 1]

        # group by x → left=min(y), right=max(y)
        unique_x = np.unique(x.astype(int))
        left_pts = []
        right_pts = []
        for xi in unique_x:
            mask = (x.astype(int) == xi)
            yy = y[mask]
            if len(yy) > 0:
                left_pts.append((xi, np.min(yy)))
                right_pts.append((xi, np.max(yy)))

        left_pts = np.array(left_pts)
        right_pts = np.array(right_pts)

        # sort by x
        left_pts = left_pts[np.argsort(left_pts[:, 0])]
        right_pts = right_pts[np.argsort(right_pts[:, 0])]

        if len(left_pts) > 5 and len(right_pts) > 5:
            xs_left = left_pts[:, 0]
            ys_left = left_pts[:, 1]
            xs_right = right_pts[:, 0]
            ys_right = right_pts[:, 1]

            spline_left = UnivariateSpline(xs_left, ys_left, s=5)
            spline_right = UnivariateSpline(xs_right, ys_right, s=5)

            xs = np.linspace(min(xs_left.min(), xs_right.min()),
                             max(xs_left.max(), xs_right.max()), 200)

            ys_left_smooth = spline_left(xs)
            ys_right_smooth = spline_right(xs)

            # left curve marker
            left_marker = Marker()
            left_marker.header.frame_id = msg.header.frame_id
            left_marker.header.stamp = self.get_clock().now().to_msg()
            left_marker.ns = "road_left"
            left_marker.id = 1
            left_marker.type = Marker.LINE_STRIP
            left_marker.action = Marker.ADD
            left_marker.scale.x = 0.1
            left_marker.color.r = 0.0
            left_marker.color.g = 0.0
            left_marker.color.b = 1.0
            left_marker.color.a = 0.8
            left_marker.lifetime = Duration(seconds=60).to_msg()

            for xi, yi in zip(xs, ys_left_smooth):
                p = Point()
                p.x = float(xi)
                p.y = float(yi)
                p.z = 0.0
                left_marker.points.append(p)

            # right curve marker
            right_marker = Marker()
            right_marker.header.frame_id = msg.header.frame_id
            right_marker.header.stamp = self.get_clock().now().to_msg()
            right_marker.ns = "road_right"
            right_marker.id = 2
            right_marker.type = Marker.LINE_STRIP
            right_marker.action = Marker.ADD
            right_marker.scale.x = 0.1
            right_marker.color.r = 1.0
            right_marker.color.g = 0.0
            right_marker.color.b = 0.0
            right_marker.color.a = 0.8
            right_marker.lifetime = Duration(seconds=60).to_msg()

            for xi, yi in zip(xs, ys_right_smooth):
                p = Point()
                p.x = float(xi)
                p.y = float(yi)
                p.z = 0.0
                right_marker.points.append(p)

            self.pub_left.publish(left_marker)
            self.pub_right.publish(right_marker)


def main(args=None):
    rclpy.init(args=args)
    node = RoadSquares()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
