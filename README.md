
# ROS 2 SLAM Frontend (Python) — Preprocessing, Feature Extraction, Scan Matching

[![Documentation Status](https://readthedocs.org/projects/slam-frontend/badge/?version=latest)](https://slam-frontend.readthedocs.io/en/latest/?badge=latest)


## Documentation

For detailed API documentation, installation guide, and pipeline architecture, see:
- **[Installation & Running Guide](https://slam-frontend.readthedocs.io/en/latest/guide/install_run.html)**
- **[Pipeline Architecture](https://slam-frontend.readthedocs.io/en/latest/guide/pipeline.html)**
- **[API Reference](https://slam-frontend.readthedocs.io/en/latest/api/index.html)**


function slam_frontend_ws {
  
  alias data_loading='ros2 launch kitti_data_loader kitti_data_loader.launch.py'
  alias foxglove='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'
  alias preprocessing='ros2 launch lidar_preprocessing preprocessing.launch.py'
  # set -e
  # --- ROS 2 and Workspace Setup ---
  VENV="${HOME}/ros2_venv"
  WS="/home/zeid/github/slam_frontend_ws"
  ROS2_DISTRO="jazzy" # Or your primary distro

  # Activate Python virtual environment
  if [ -f "${VENV}/bin/activate" ]; then
    source "${VENV}/bin/activate"
    echo "✅ ROS 2 + venv ready."
  fi

  # Source ROS 2
  if [ -f "/opt/ros/${ROS2_DISTRO}/setup.zsh" ]; then
    source "/opt/ros/${ROS2_DISTRO}/setup.zsh"
  fi

  # Source the workspace
  if [ -f "${WS}/install/setup.zsh" ]; then
    source "${WS}/install/setup.zsh"
  fi

  # Setup argcomplete after everything is sourced
  if command -v register-python-argcomplete >/dev/null; then
      eval "$(register-python-argcomplete ros2)"
      eval "$(register-python-argcomplete colcon)"
  fi

  cd ${WS}
}
# call the function
slam_frontend_ws

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
