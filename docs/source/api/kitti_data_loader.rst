==============================
ROS 2 KITTI Data Loader Node
==============================

Overview
--------

This script implements a ROS 2 node, ``kitti_publisher``, designed to read and "play back" data from the KITTI Odometry dataset. It loads a specified data sequence, reads the Velodyne LiDAR scans, ground-truth poses, and calibration files, and then publishes this data as ROS 2 messages and TF transforms at a configurable rate. The node's primary purpose is to provide a realistic simulation of KITTI sensor data within the ROS 2 ecosystem, paying special attention to correcting coordinate frames for compatibility with ROS standards (REP-103).

Core Functionality
------------------

The node's core features include:

* **Data Loading**: Loads data from a standard KITTI Odometry dataset layout (``data_odometry_velodyne``, ``data_odometry_poses``, ``data_odometry_calib``).
* **LiDAR Publishing**: Reads Velodyne ``.bin`` files, converts them to ``sensor_msgs/msg/PointCloud2``, and publishes them using the ``qos_profile_sensor_data`` profile.
* **Pose Publishing**: Reads the ground-truth pose ``.txt`` file and publishes the vehicle's pose as a ``geometry_msgs/msg/PoseStamped`` message and a dynamic TF transform.
* **Static Transform Publishing**: Reads the sequence's ``calib.txt`` file to determine the extrinsic relationship between the vehicle's base and the LiDAR sensor, publishing it as a single, latched (static) TF transform.
* **Robust Checks**: Includes file and directory existence checks to provide clear error messages.
* **Rate Control**: Uses an ``rclpy`` timer to publish data at a configurable frequency (``publish_rate_hz``).

Coordinate Frame Semantics
~~~~~~~~~~~~~~~~~~~~~~~~~~

A critical function of this node is to handle the differences between KITTI's coordinate systems and the ROS REP-103 standard.

.. note::
   * **ROS (REP-103)**: ``x`` forward, ``y`` left, ``z`` up
   * **KITTI Velodyne**: ``x`` forward, ``y`` left, ``z`` up (This **matches** ROS, so point cloud data is published **without modification**.)
   * **KITTI Camera (cam0)**: ``x`` right, ``y`` down, ``z`` forward (This is the frame used for ground-truth poses.)

The node defines a rotation matrix to convert data from the KITTI Camera frame to the ROS frame. This conversion is applied to:

1.  **Ground-Truth Poses**: Poses from ``<SEQ>.txt`` are transformed to become ``T_map_base_link`` in ROS coordinates. 
2.  **Calibration Data**: The ``Tr_velo_to_cam`` matrix is used to calculate the correct static transform from the ROS-compliant ``base_link`` to the ``velodyne`` frame.

If calibration data is not found, an identity transform is published.

ROS 2 Interface
---------------

**Parameters:**

.. list-table:: KITTI Publisher Parameters
   :header-rows: 1
   :widths: 30 50 70

   * - **Parameter**
     - **Default Value (from params.yaml)**
     - **Meaning**
   * - ``kitti_data_dir``
     - ``/home/zeidk/github/slam_frontend_ws/data/kitti``
     - Absolute path to the root KITTI dataset directory.
   * - ``dataset_sequence``
     - ``s00``
     - Identifier of the KITTI dataset sequence to publish (e.g., "00" or "s00").
   * - ``publish_rate_hz``
     - ``10.0``
     - Publishing frequency of KITTI sensor data in hertz.
   * - ``pointcloud_topic``
     - ``/kitti/pointcloud_raw``
     - Topic name for raw LiDAR point clouds.
   * - ``ground_truth_pose_topic``
     - ``/kitti/ground_truth_pose``
     - Topic name for ground-truth vehicle poses (PoseStamped).
   * - ``map_frame_id``
     - ``map``
     - Frame ID for the global "map" coordinate system.
   * - ``base_frame_id``
     - ``base_link``
     - Frame ID representing the vehicle body/base.
   * - ``lidar_frame_id``
     - ``velodyne``
     - Frame ID for the LiDAR sensor.
   * - ``base_equals_cam0``
     - ``true``
     - If True, assumes `base_link` coincides with KITTI's `cam0` frame for pose conversion.
   * - ``use_cam0_poses``
     - ``true``
     - If True, reads and publishes the ground-truth poses from the ``poses/<SEQ>.txt`` file.

**Subscriptions:**

This node does not have any subscriptions; it only publishes data.

**Publications:**

* ``<pointcloud_topic>`` (``sensor_msgs/msg/PointCloud2``):
    * Publishes the Velodyne LiDAR scan using ``qos_profile_sensor_data``.
* ``<ground_truth_pose_topic>`` (``geometry_msgs/msg/PoseStamped``):
    * Publishes the vehicle's ground-truth pose.
* ``/tf`` (``tf2_msgs/msg/TFMessage``):
    * Broadcasts the dynamic ``map_frame_id`` -> ``base_frame_id`` transform.
* ``/tf_static`` (``tf2_msgs/msg/TFMessage``):
    * Broadcasts the static ``base_frame_id`` -> ``lidar_frame_id`` transform based on calibration data.

**Launch Files:**

**File:** `src/kitti_data_loader/launch/kitti_data_loader.launch.py`

This launch file starts the KITTI publisher node with parameters loaded from
the configuration file. It also allows all parameters to be overridden from
the command line.

.. literalinclude:: ../../../src/kitti_data_loader/launch/kitti_data_loader.launch.py
   :language: python
   :caption: kitti_data_loader/launch/kitti_data_loader.launch.py
   :linenos:

Compile and Run
-----------------

**1. Configure Dataset Path**

Before launching, you must update the ``kitti_data_loader/config/params.yaml`` file to point to the absolute path of your KITTI data directory.
Edit the default path for ``kitti_data_dir`` parameter:

.. code-block:: yaml
   :caption: kitti_data_loader/config/params.yaml
   :emphasize-lines: 2

   kitti_data_dir: "/home/zeidk/github/slam_frontend_ws/data/kitti" # change the path
   # ... other parameters

**2. Build the Workspace**

From the root of your ROS 2 workspace, build the package:

.. code-block:: bash

   colcon build --symlink-install
   source install/setup.bash

**3. Run and Visualize**

You will need two sourced ROS 2 terminals and the Foxglove Studio application. 

**Terminal 1: Start Foxglove Bridge**
Launch the websocket bridge that Foxglove Studio uses to communicate with ROS 2.

.. code-block:: bash

   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

**Foxglove Studio GUI**

1. Open Foxglove Studio.
2. Navigate to **Open Connection** -> **Foxglove WebSocket**.
3. Ensure the URL is ``ws://localhost:8765`` and click **Open**.
4. Load the pre-configured layout: **Layouts** -> **Personal** -> **Add**, and navigate to your workspace to select ``kitti_data_loader/config/kitti_data_loader_layout.json``.

**Terminal 2: Launch the KITTI Publisher**
Once the bridge and Foxglove are running and connected, launch the publisher node:

.. code-block:: bash

   ros2 launch kitti_data_loader kitti_data_loader.launch.py



Module
--------

.. automodule:: kitti_data_loader.kitti_publisher
   :members:
   :undoc-members:
   :show-inheritance:
