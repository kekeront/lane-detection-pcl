# Architecture

## Pipeline Overview

```
LiDAR Sensor
     │
     ▼
┌─────────────────────────────────┐
│  /sensor/lidar_front/points     │  PointCloud2 (raw 3D points)
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│  ransac_node.py                 │  Ground-plane filter (RANSAC-ready)
│  ─ Filters road surface by Z   │  Quantizes to 0.5m grid cells
│  ─ Publishes grid as CUBE_LIST │
└────────────┬────────────────────┘
             │
     ┌───────┴───────┐
     ▼               ▼
┌──────────┐  ┌──────────────────┐
│ curve.py │  │ Path Planners    │
│          │  │ ─ astar.py       │
│ Spline   │  │ ─ bfs.py         │
│ fitting  │  │ ─ dfs.py         │
│ for lane │  │ ─ djikstra.py    │
│ boundary │  │                  │
│ markers  │  │ Grid → Path      │
└────┬─────┘  └────────┬─────────┘
     │                 │
     ▼                 ▼
┌─────────────────────────────────┐
│  RViz2 Visualization            │
│  ─ /semantic_map/squares (grid) │
│  ─ /semantic_map/road_left      │
│  ─ /semantic_map/road_right     │
│  ─ Path overlays (LINE_STRIP)   │
└─────────────────────────────────┘
```

## Node Descriptions

| Node | File | Purpose |
|------|------|---------|
| `ransac_node` | `ransac_node.py` | Core road segmentation. Filters ground-plane points and publishes a semantic grid map. |
| `curve` | `curve.py` | Extends ransac_node. Fits UnivariateSpline curves to left/right lane boundaries. |
| `astar` | `astar.py` | A* path planning on the road grid. Uses Euclidean distance heuristic. |
| `bfs` | `bfs.py` | Breadth-first search path planning. Finds shortest unweighted path. |
| `dfs` | `dfs.py` | Depth-first search path planning. Explores one branch fully before backtracking. |
| `djikstra` | `djikstra.py` | Dijkstra's algorithm. Uniform-cost search on the grid. |

## Data Flow

1. **Input**: Raw `PointCloud2` from front-facing LiDAR
2. **Filter**: Z-axis threshold (< 0.2m) isolates ground-plane points. Designed to be replaced with full RANSAC plane fitting.
3. **Quantize**: Continuous 3D points → discrete 0.5m grid cells
4. **Output**: Grid cells published as RViz markers; path planners compute routes across the grid
