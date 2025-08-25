# Road Squares ROS2 Node

A ROS2 node that processes LiDAR point cloud data to create a semantic grid map of road surfaces, visualized as square tiles in RViz.

## Overview

This node subscribes to LiDAR point cloud data, identifies road surface points, and publishes a grid-based semantic map representation. The road surface is divided into square tiles and visualized as green transparent markers in RViz.

## Features

- Real-time LiDAR point cloud processing
- Road surface segmentation (currently using simple z-height filtering)
- Grid-based semantic mapping with configurable square size
- RViz visualization with colored markers
- Ready for integration with RANSAC-based road segmentation

## Requirements

### Dependencies
- ROS2 (tested with appropriate ROS2 distribution)
- Python 3
- Required Python packages:
  - `rclpy`
  - `sensor_msgs`
  - `visualization_msgs` 
  - `geometry_msgs`
  - `sensor_msgs_py`
  - `numpy`

### System Requirements
- ROS2 workspace properly configured
- LiDAR sensor publishing to `/sensor/lidar_front/points`

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/kekeront/lane-detection-pcl
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

3. Install Python dependencies:
```bash
pip install numpy
```

## Usage

### Running the Node

```bash
ros2 run ransac_seg ransac_node.py
```

Or launch directly:
```bash
python3 ransac_node.py
```

### Topics

#### Subscribed Topics
- `/sensor/lidar_front/points` (`sensor_msgs/PointCloud2`)
  - Input LiDAR point cloud data

#### Published Topics
- `/semantic_map/squares` (`visualization_msgs/Marker`)
  - Grid squares representing detected road surface areas

## Configuration

### Grid Parameters
You can modify the following parameters in the code:

- `square_size`: Size of each grid square in meters (default: 1.0m)
- `half_size`: Half the square size for centering calculations

### Road Detection Parameters
- Currently uses simple z-height filtering (`z < 0.2m`)
- **TODO**: Replace with RANSAC-based road segmentation for better accuracy

### Visualization Parameters
- Square thickness: 0.05m (thin tiles)
- Color: Green with 50% transparency
- Frame: Inherits from input point cloud frame

## Visualization in RViz

1. Launch RViz:
```bash
rviz2
```

2. Add a Marker display
3. Set the topic to `/semantic_map/squares`
4. Configure the fixed frame to match your LiDAR frame

The road squares will appear as green transparent tiles on the detected road surface.

## Code Structure

### Main Components

- **`RoadSquares`**: Main ROS2 node class
- **`ros_to_numpy()`**: Utility function to convert ROS PointCloud2 messages to numpy arrays
- **`callback()`**: Main processing function that handles incoming point cloud data

### Processing Pipeline

1. Convert ROS PointCloud2 to numpy array
2. Filter points for road surface detection
3. Quantize road points into grid cells
4. Create visualization markers for detected grid squares
5. Publish markers for RViz display

## Development Notes

### Current Limitations
- Uses simple z-height filtering instead of robust road segmentation
- Fixed grid parameters (not configurable via ROS parameters)
- Single-frame processing (no temporal consistency)

### Planned Improvements
- [ ] Integrate RANSAC-based road plane segmentation
- [ ] Add ROS parameter server for runtime configuration
- [ ] Implement temporal filtering for grid stability
- [ ] Add support for multiple road surfaces/elevations
- [ ] Performance optimization for large point clouds

### Integration with RANSAC
The code includes a TODO comment where existing RANSAC road segmentation should be integrated:

```python
# ✅ TODO: Apply your existing RANSAC road segmentation here
# For now, I assume you already have road vs obstacle classification
# Let's take a simple filter (ground plane / road_surface)
road_pts = pts[pts[:,2] < 0.2]   # crude filter by z (replace with your RANSAC output)
```

Replace the simple z-filtering with your RANSAC implementation for better road detection accuracy.

## Troubleshooting

### Common Issues

1. **No squares visible in RViz**
   - Check that LiDAR data is being published to `/sensor/lidar_front/points`
   - Verify the frame_id matches between LiDAR and RViz fixed frame
   - Ensure points exist below the z-height threshold (0.2m)

2. **Performance issues with large point clouds**
   - Consider downsampling the input point cloud
   - Implement spatial filtering to process only nearby points
   - Optimize the grid quantization algorithm

3. **Incorrect grid positioning**
   - Verify the coordinate system and frame transformations
   - Check that the grid quantization logic matches your coordinate conventions

## License

The code is protected under GNU GENERAL PUBLIC LICENSE

## Contributing

We welcome contributions from the community! 🎉 Whether it's fixing a bug, adding new features, or improving documentation, your help is appreciated.

### How to Contribute

1. **Check Existing Issues**  
   Look through the [Issues](../../issues) tab to see if your idea or bug report already exists.  
   - If it does, feel free to join the discussion.  
   - If not, open a **new issue** to describe your suggestion or bug.

2. **Fork the Repository**  
   Create your own fork of this project and clone it locally.

   ```bash
   git clone https://github.com/your-username/endtermv1.git
   cd endtermv1
3. **Create a New Branch**
    Always make changes in a new branch.

```bash
git checkout -b feature/your-feature-name
```
4. **Make Your Changes**
* Follow existing code style and structure.
* Add comments where necessary.
* Test your changes before committing.
5. **Commit and Push**

```bash
git add .
git commit -m "Add: short description of your change"
git push origin feature/your-feature-name
```
6. **Open a Pull Request (PR)**

* Go to the original repository and click New Pull Request.
* Clearly describe what you’ve changed and why.
* Reference related issues (if any).
7. **Review Process**
* A maintainer will review your PR.
* They may suggest changes — please be responsive.
* Once approved ✅, your PR will be merged.

8. **Contribution Guidelines**
* Keep commits small and focused.
* Use descriptive commit messages.
* Write clean, readable, and well-documented code.
* Be respectful and collaborative in discussions.

Thank you for helping improve this project! 🚀
