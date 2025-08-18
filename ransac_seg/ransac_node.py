#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d
import math
from geometry_msgs.msg import Point
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
from scipy import ndimage
import cv2

def ros_to_numpy(msg):
    return np.array([
        [p[0], p[1], p[2]]
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ], dtype=np.float32)


class EnhancedLaneDetectionNode(Node):
    def __init__(self):
        super().__init__('enhanced_lane_detection_node')

        # Subscriptions
        self.sub_front = self.create_subscription(
            PointCloud2,
            '/sensor/lidar_front/points',
            self.callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(MarkerArray, "/lane_detection_markers", 10)

        # RANSAC parameters for plane detection (relaxed for more points)
        self.distance_threshold = 0.1
        self.num_iterations = 100

        # Height filtering parameters (will be adapted based on ground plane)
        self.z_tolerance_below = 2.0
        self.z_tolerance_above = 0.3

        # Minimum points threshold (adjustable)
        self.min_road_points = 1

        # Lane detection params
        self.lane_width_min = 0.0
        self.lane_width_max = 4.0
        self.roi_distance = 20.0
        self.roi_width = 2.0

        # Clustering parameters
        self.dbscan_eps = 0.3
        self.dbscan_min_samples = 5

        # Grid-based intensity analysis
        self.grid_resolution = 0.2

        self.get_logger().info("Enhanced Lane Detection Node initialized")

    def callback(self, msg):
        try:
            pts = ros_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse PointCloud2: {e}")
            return

        if pts.shape[0] < 100:
            self.get_logger().warn("Too few points for processing")
            return

        # Step 1: Ground plane detection using RANSAC
        ground_plane, road_points, non_road_points = self.detect_ground_plane(pts)

        if road_points.shape[0] < self.min_road_points:
            self.get_logger().warn(f"Insufficient road points: {road_points.shape[0]} < {self.min_road_points}")
            # Try with more relaxed parameters
            backup_distance_threshold = self.distance_threshold
            backup_z_below = self.z_tolerance_below
            backup_z_above = self.z_tolerance_above
            self.distance_threshold = 0.25
            self.z_tolerance_below = 1.5
            self.z_tolerance_above = 2.0
            ground_plane, road_points, non_road_points = self.detect_ground_plane(pts)
            # restore
            self.distance_threshold = backup_distance_threshold
            self.z_tolerance_below = backup_z_below
            self.z_tolerance_above = backup_z_above

            if road_points.shape[0] < self.min_road_points:
                self.get_logger().warn(f"Still insufficient road points after retry: {road_points.shape[0]}")
                return

        # Step 2: ROI
        roi_points = self.filter_roi(road_points)

        # Step 3: lane boundaries
        left_lane, right_lane, lane_markers = self.detect_lane_boundaries(roi_points)

        # Step 4: create visualization markers
        markers = self.create_visualization_markers(
            msg, road_points, non_road_points, roi_points, left_lane, right_lane, lane_markers, ground_plane
        )

        self.pub.publish(markers)

    def detect_ground_plane(self, pts):
        """Robust ground plane detection with horizontality constraint and fallback."""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)

        best = None
        remaining = pcd
        candidates = []
        max_trials = 3

        for _ in range(max_trials):
            if len(remaining.points) < 500:
                break
            try:
                model, inliers = remaining.segment_plane(
                    distance_threshold=self.distance_threshold,
                    ransac_n=3,
                    num_iterations=self.num_iterations
                )
            except Exception as e:
                self.get_logger().warn(f"Plane segmentation failed: {e}")
                break

            a, b, c, d = model
            inlier_idx = np.array(inliers, dtype=np.int32)
            inlier_pts = np.asarray(remaining.points)[inlier_idx]
            candidates.append((abs(c), len(inliers), (a, b, c, d), inlier_pts))

            mask = np.ones(len(remaining.points), dtype=bool)
            mask[inlier_idx] = False
            remaining = o3d.geometry.PointCloud(
                o3d.utility.Vector3dVector(np.asarray(remaining.points)[mask])
            )

        if candidates:
            candidates.sort(key=lambda t: (t[0], t[1]), reverse=True)
            _, _, plane_model, inlier_pts = candidates[0]
            a, b, c, d = plane_model
        else:
            self.get_logger().warn("No RANSAC plane found; using low-Z fallback.")
            z10 = np.percentile(pts[:, 2], 10)
            z_min = z10 - 0.3
            z_max = z10 + 0.3
            road_points = pts[(pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)]
            non_road_points = pts[(pts[:, 2] < z_min) | (pts[:, 2] > z_max)]
            return (0, 0, 1, -z10), road_points, non_road_points

        self.get_logger().info(f"Plane: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0 (|c|={abs(c):.2f})")

        if abs(c) < 0.6:
            self.get_logger().warn("Best plane not horizontal (|c|<0.6); using low-Z fallback.")
            z10 = np.percentile(pts[:, 2], 10)
            z_min = z10 - 0.35
            z_max = z10 + 0.35
            road_points = pts[(pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)]
            non_road_points = pts[(pts[:, 2] < z_min) | (pts[:, 2] > z_max)]
            return (0, 0, 1, -z10), road_points, non_road_points

        ground_height = -d / c
        tol_below = max(0.3, float(self.z_tolerance_below))
        tol_above = max(0.3, float(abs(getattr(self, "z_tolerance_above", 0.5))))
        z_min = ground_height - tol_below
        z_max = ground_height + tol_above
        road_mask = (inlier_pts[:, 2] >= z_min) & (inlier_pts[:, 2] <= z_max)
        road_points = inlier_pts[road_mask]

        outlier_pts = pts
        lowz_mask = (outlier_pts[:, 2] >= z_min) & (outlier_pts[:, 2] <= z_max)
        extra_lowz = outlier_pts[lowz_mask]
        if extra_lowz.size:
            road_points = np.vstack([road_points, extra_lowz])

        non_road_points = pts[(pts[:, 2] < z_min) | (pts[:, 2] > z_max)]
        self.get_logger().info(
            f"Ground z≈{ground_height:.2f} band=[{z_min:.2f},{z_max:.2f}] "
            f"road={road_points.shape[0]} nonroad={non_road_points.shape[0]}"
        )
        return (a, b, c, d), road_points, non_road_points

    def filter_roi(self, points):
        mask = (
            (points[:, 0] > 0) & (points[:, 0] < self.roi_distance) &
            (points[:, 1] > -self.roi_width / 2) & (points[:, 1] < self.roi_width / 2)
        )
        roi_points = points[mask]
        self.get_logger().info(f"ROI points: {roi_points.shape[0]}")
        return roi_points

    def detect_lane_boundaries(self, points):
        if points.shape[0] < 20:
            return None, None, []

        left_boundary_grid, right_boundary_grid = self.grid_based_detection(points)
        left_boundary_cluster, right_boundary_cluster = self.clustering_based_detection(points)
        left_boundary_edge, right_boundary_edge = self.edge_based_detection(points)

        left_lane = left_boundary_grid if left_boundary_grid is not None else left_boundary_cluster
        right_lane = right_boundary_grid if right_boundary_grid is not None else right_boundary_cluster

        lane_markers = []
        if left_lane is not None:
            lane_markers.append(('left', left_lane))
        if right_lane is not None:
            lane_markers.append(('right', right_lane))
        return left_lane, right_lane, lane_markers

    def grid_based_detection(self, points):
        if points.shape[0] < 10:
            return None, None
        x_min, x_max = 0, self.roi_distance
        y_min, y_max = -self.roi_width / 2, self.roi_width / 2
        x_bins = max(1, int((x_max - x_min) / self.grid_resolution))
        y_bins = max(1, int((y_max - y_min) / self.grid_resolution))
        hist, x_edges, y_edges = np.histogram2d(
            points[:, 0], points[:, 1],
            bins=[x_bins, y_bins],
            range=[[x_min, x_max], [y_min, y_max]]
        )
        hist_smooth = ndimage.gaussian_filter(hist, sigma=1.0)
        left_boundary = []
        right_boundary = []
        for i in range(hist_smooth.shape[0]):
            slice_density = hist_smooth[i, :]
            if np.max(slice_density) < 2:
                continue
            peaks = self.find_peaks_1d(slice_density)
            if len(peaks) >= 2:
                x_coord = (x_edges[i] + x_edges[i + 1]) / 2.0
                left_peak_idx = peaks[0]
                right_peak_idx = peaks[-1]
                left_y = (y_edges[left_peak_idx] + y_edges[left_peak_idx + 1]) / 2.0
                right_y = (y_edges[right_peak_idx] + y_edges[right_peak_idx + 1]) / 2.0
                if abs(right_y - left_y) > self.lane_width_min:
                    left_boundary.append([x_coord, left_y, 0.0])
                    right_boundary.append([x_coord, right_y, 0.0])
        left_lane = np.array(left_boundary) if left_boundary else None
        right_lane = np.array(right_boundary) if right_boundary else None
        return left_lane, right_lane

    def clustering_based_detection(self, points):
        if points.shape[0] < 20:
            return None, None
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
        labels = clustering.fit_predict(points[:, :2])
        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            cluster_points = points[labels == label]
            if len(cluster_points) > 10:
                clusters.append(cluster_points)
        if len(clusters) < 2:
            return None, None
        clusters.sort(key=lambda c: np.mean(c[:, 1]))
        left_cluster = clusters[0]
        right_cluster = clusters[-1]
        left_lane = self.fit_line_to_cluster(left_cluster)
        right_lane = self.fit_line_to_cluster(right_cluster)
        return left_lane, right_lane

    def edge_based_detection(self, points):
        if points.shape[0] < 30:
            return None, None
        # Placeholder: could project to image and run Canny etc.
        return None, None

    def fit_line_to_cluster(self, cluster_points):
        if len(cluster_points) < 5:
            return None
        sorted_points = cluster_points[np.argsort(cluster_points[:, 0])]
        x = sorted_points[:, 0].reshape(-1, 1)
        y = sorted_points[:, 1]
        try:
            ransac = RANSACRegressor(residual_threshold=0.1, random_state=42)
            ransac.fit(x, y)
            x_line = np.linspace(np.min(x), np.max(x), 20)
            y_line = ransac.predict(x_line.reshape(-1, 1))
            line_points = np.column_stack([x_line, y_line, np.zeros_like(x_line)])
            return line_points
        except Exception as e:
            self.get_logger().warn(f"Line fitting failed: {e}")
            return None

    def find_peaks_1d(self, signal, min_height=1):
        peaks = []
        for i in range(1, len(signal) - 1):
            if (signal[i] > signal[i - 1] and signal[i] > signal[i + 1] and signal[i] > min_height):
                peaks.append(i)
        return peaks

    def create_visualization_markers(self, msg, road_points, non_road_points, roi_points, left_lane, right_lane, lane_markers, ground_plane):
        markers = MarkerArray()
        marker_id = 0

        if road_points is not None and road_points.shape[0] > 0:
            road_marker = self.create_point_marker(msg, marker_id, "road_surface", road_points, color=(0, 1, 0, 0.8), scale=0.08)
            markers.markers.append(road_marker)
            marker_id += 1

        if non_road_points is not None and non_road_points.shape[0] > 0:
            non_road_marker = self.create_point_marker(msg, marker_id, "obstacles", non_road_points, color=(1, 0, 0, 1.0), scale=0.12)
            markers.markers.append(non_road_marker)
            marker_id += 1

        roi_boundary_marker = self.create_roi_boundary_marker(msg, marker_id)
        markers.markers.append(roi_boundary_marker)
        marker_id += 1

        if left_lane is not None:
            left_marker = self.create_lane_marker(msg, marker_id, "left_lane", left_lane, color=(1, 1, 0, 1.0))
            markers.markers.append(left_marker)
            marker_id += 1

        if right_lane is not None:
            right_marker = self.create_lane_marker(msg, marker_id, "right_lane", right_lane, color=(0, 1, 1, 1.0))
            markers.markers.append(right_marker)
            marker_id += 1

        plane_marker = self.create_plane_marker(msg, marker_id, ground_plane)
        markers.markers.append(plane_marker)
        return markers

    def create_point_marker(self, msg, marker_id, ns, points, color, scale):
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale * 0.5
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.points = [self.make_point(p) for p in points]
        return marker

    def create_lane_marker(self, msg, marker_id, ns, lane_points, color):
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.points = [self.make_point(p) for p in lane_points]
        return marker

    def create_roi_boundary_marker(self, msg, marker_id):
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "roi_boundary"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.7
        roi_points = [
            [0, -self.roi_width / 2, 0],
            [self.roi_distance, -self.roi_width / 2, 0],
            [self.roi_distance, self.roi_width / 2, 0],
            [0, self.roi_width / 2, 0],
            [0, -self.roi_width / 2, 0],
        ]
        marker.points = [self.make_point(p) for p in roi_points]
        return marker

    def create_plane_marker(self, msg, marker_id, plane_model):
        a, b, c, d = plane_model
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ground_plane"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        norm = math.sqrt(a * a + b * b + c * c)
        if c < 0:
            a, b, c, d = -a, -b, -c, -d
        cx = -a * d / (norm * norm)
        cy = -b * d / (norm * norm)
        cz = -c * d / (norm * norm)
        marker.pose.position.x = float(cx)
        marker.pose.position.y = float(cy)
        marker.pose.position.z = float(cz)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 8.0
        marker.scale.y = 8.0
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.2
        return marker

    def make_point(self, arr):
        p = Point()
        p.x, p.y, p.z = float(arr[0]), float(arr[1]), float(arr[2])
        return p


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedLaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
