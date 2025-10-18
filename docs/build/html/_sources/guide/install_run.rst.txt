Installation and Running
========================

This guide explains how to install prerequisites, obtain the KITTI dataset, launch
the pipeline, and visualize results in Foxglove Studio.

Prerequisites
-------------

- Ubuntu 22.04 or later
- ROS 2 (Humble, Iron, or Jazzy)
- Python 3.8+
- Colcon

Install Dependencies
--------------------

Foxglove Bridge:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-$ROS_DISTRO-foxglove-bridge

(Optional) Python tools used elsewhere in this project:

.. code-block:: bash

   python3 -m pip install --upgrade pip
   python3 -m pip install numpy open3d

Build the Workspace
-------------------

.. code-block:: bash

   cd ~/ros2_slam_demo_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash

Download KITTI Odometry Data
----------------------------

Obtain the following archives from the KITTI Odometry page:

- ``data_odometry_velodyne.zip`` (required)
- ``data_odometry_calib.zip`` (optional)
- ``data_odometry_poses.zip`` (optional)

Unpack so that sequence 00 is available as:

.. code-block:: text

   dataset/sequences/00/velodyne/000000.bin

Run the Pipeline
----------------

Start the Foxglove Bridge:

.. code-block:: bash

   ros2 launch foxglove_bridge foxglove_bridge_launch.xml

Launch the full SLAM demo:

.. code-block:: bash

   ros2 launch slam_pipeline demo_pipeline.launch.py

Visualize in Foxglove Studio
----------------------------

1. Start Foxglove Studio (desktop app or web).
2. Open a connection using **Foxglove WebSocket** at ``ws://localhost:8765``.
3. Add a 3D panel and subscribe to:

   - ``/points_raw``
   - ``/points_filtered``
   - ``/keypoints/corners``
   - ``/keypoints/planes``
   - ``/points_aligned``
   - ``/odom``
