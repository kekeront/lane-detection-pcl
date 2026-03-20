# lane-detection-pcl

A **ROS 2** package (`ransac_seg`) for road surface segmentation and lane boundary detection using **LiDAR point cloud data (PCL)**. The package processes `PointCloud2` messages, applies ground-plane filtering (with a RANSAC-ready interface), and publishes semantic road markers and smooth lane curves to RViz.

---

## Overview

This package subscribes to a front LiDAR topic, filters road surface points, and:
- Overlays the road surface as a **grid of cubes** (semantic map squares)
- Fits **spline curves** to the left and right lane boundaries
- Provides multiple **path-planning algorithm nodes** (A*, BFS, DFS, Dijkstra) operating on the road grid

---

## Package Structure

```
lane-detection-pcl/
├── ransac_seg/
│   ├── ransac_node.py     # Core road segmentation node (RANSAC-ready)
│   ├── curve.py           # Lane boundary curve fitting with splines
│   ├── astar.py           # A* path planning on road grid
│   ├── bfs.py             # BFS path planning on road grid
│   ├── dfs.py             # DFS path planning on road grid
│   └── djikstra.py        # Dijkstra path planning on road grid
├── launch/
│   └── algorithms.launch.py   # Launch file for all nodes
├── package.xml
├── setup.py
└── setup.cfg
```

---

## ROS 2 Topics

### Subscribed
| Topic | Type | Description |
|---|---|---|
| `/sensor/lidar_front/points` | `sensor_msgs/PointCloud2` | Raw LiDAR point cloud input |

### Published
| Topic | Type | Description |
|---|---|---|
| `/semantic_map/squares` | `visualization_msgs/Marker` | Road surface grid (CUBE_LIST) |
| `/semantic_map/road_left` | `visualization_msgs/Marker` | Left lane boundary spline (LINE_STRIP) |
| `/semantic_map/road_right` | `visualization_msgs/Marker` | Right lane boundary spline (LINE_STRIP) |

---

## Dependencies

- **ROS 2** (tested with `ament_python` build type)
- `rclpy`
- `sensor_msgs`, `sensor_msgs_py`
- `visualization_msgs`
- `geometry_msgs`
- `numpy`
- `scipy` (for `UnivariateSpline` lane curve fitting)

---

## Installation

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/kekeront/lane-detection-pcl.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select ransac_seg

# Source
source install/setup.bash
```

---

## Usage

### Run a single node

```bash
# Road segmentation + grid map
ros2 run ransac_seg ransac_node

# Road segmentation + grid map + lane curves
ros2 run ransac_seg curve

# Path planning
ros2 run ransac_seg astar
ros2 run ransac_seg bfs
ros2 run ransac_seg dfs
ros2 run ransac_seg djikstra
```

### Launch all nodes

```bash
ros2 launch ransac_seg algorithms.launch.py
```

---

## Visualization

Open **RViz2** and add the following **Marker** displays:
- `/semantic_map/squares` — green road surface tiles
- `/semantic_map/road_left` — blue left lane boundary
- `/semantic_map/road_right` — red right lane boundary

---

## License

This project is licensed under the **GNU General Public License v3.0**. See [LICENSE](LICENSE) for details.
