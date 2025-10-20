KITTI Data Loader
=================

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
-------

.. automodule:: kitti_data_loader.kitti_publisher
   :members:
   :undoc-members:
   :show-inheritance:

Launch Files
-------------

This section describes the launch files used to start the nodes in this workspace.

`kitti_data_loader.launch.py`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** `src/kitti_data_loader/launch/kitti_data_loader.launch.py`

This launch file starts the KITTI publisher node with parameters loaded from
the configuration file. It also allows all parameters to be overridden from
the command line. 

.. literalinclude:: ../../../src/kitti_data_loader/launch/kitti_data_loader.launch.py
   :language: python
   :caption: kitti_data_loader/launch/kitti_data_loader.launch.py
   :linenos:


Parameters
--------------

.. literalinclude:: ../../../src/kitti_data_loader/config/params.yaml
   :language: yaml
   :caption: kitti_data_loader/config/params.yaml
   :linenos:

Description
~~~~~~~~~~~

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

.. If you have more than one YAML in this package, repeat:
..  .. literalinclude:: ../../../src/lidar_preprocessing/config/another.yaml
..     :language: yaml
..     :caption: lidar_preprocessing/config/another.yaml
..     :linenos: