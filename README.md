
# ROS 2 SLAM Frontend (Python) — Preprocessing, Feature Extraction, Scan Matching

[![Documentation Status](https://readthedocs.org/projects/slam-frontend/badge/?version=latest)](https://slam-frontend.readthedocs.io/en/latest/?badge=latest)


## Documentation

For detailed API documentation, installation guide, and pipeline architecture, see:
- **[Documentation](https://slam-frontend.readthedocs.io/en/latest/index.html)**



<!-- ## Workspace

This workspace contains three ROS 2 Python packages aligned with lecture 2C (SLAM Frontend) sections:
- `slam_preprocessing`: filters and deskews raw point clouds
- `slam_feature_extraction`: extracts features (e.g., corners/planes) from filtered clouds
- `slam_scan_matching`: aligns consecutive scans (ICP/NDT) and publishes odometry

**Topics** (by default; configurable via parameters):
- Input raw cloud: `/points_raw` (`sensor_msgs/msg/PointCloud2`)
- Filtered cloud: `/points_filtered`
- Keypoints (corners, planes): `/keypoints/corners`, `/keypoints/planes` (`sensor_msgs/msg/PointCloud2`)
- Aligned/registered cloud: `/points_aligned`
- Odometry: `/odom` (`nav_msgs/msg/Odometry`)

## Quick start
```bash
# From this workspace root
colcon build --symlink-install
source install/setup.bash

# Run the end‑to‑end pipeline (namespaced demo)
ros2 launch slam_pipeline demo_pipeline.launch.py

# (Optional) Play a ROS 2 bag with point clouds in /points_raw
ros2 bag play <your_bag>
```

## Visualize in Foxglove Studio
1. Install and run the Foxglove bridge:
   ```bash
   sudo apt install ros-$ROS_DISTRO-foxglove-bridge
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   ```
2. In Foxglove Studio: **Open connection → Foxglove WebSocket**, URL `ws://localhost:8765`.
3. Add **3D** and **Plot** panels, subscribe to `/points_filtered`, `/keypoints/*`, `/points_aligned`, and `/odom`. -->
