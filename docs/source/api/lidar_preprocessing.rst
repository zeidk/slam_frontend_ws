
==================================
ROS 2 Voxel Grid Downsampler Node
==================================

Overview
--------

This script implements a ROS 2 node, ``voxel_downsampler``, that subscribes to a ``sensor_msgs/msg/PointCloud2`` topic, applies a voxel-grid downsampling filter, and publishes the resulting, smaller point cloud.

The node is designed for robustness and simplicity, using only **NumPy** for its computations. It can handle arbitrary input point cloud field layouts and ensures the output is always a sanitized, consistent ``PointCloud2`` message with the fields ``[x, y, z, intensity]`` (all ``FLOAT32``).

Core Functionality
------------------

The node's processing pipeline, triggered on each incoming message, is as follows:

1.  **Robust Parsing**: The input ``sensor_msgs/msg/PointCloud2`` message is converted into an (N, 4) NumPy array.

    * It uses the ``sensor_msgs_py.point_cloud2.read_points`` utility to correctly handle different field names, data types, and memory offsets in the input cloud.
    * It explicitly looks for ``x``, ``y``, ``z``, and ``intensity``.
    * If an ``intensity`` field is not found, it is **synthesized** with a default value of ``1.0`` to maintain a consistent [x, y, z, intensity] structure.

2.  **Sanitization**: All points (rows) containing ``NaN`` or ``Inf`` values are removed.

3.  **Ground Filtering (Optional)**: If the ``filter_ground`` parameter is set to ``True``, any point with a ``z`` coordinate less than or equal to the ``ground_threshold`` parameter is discarded.

4.  **Voxel Downsampling**: The core filtering logic is applied.

    * Points are discretized into integer voxel indices based on the ``voxel_size`` parameter.
    * The node computes the **centroid** (mean) of all point attributes [x, y, z, intensity] within each unique voxel.
    * Optionally discards voxels that contain fewer than ``min_points_per_voxel`` points.

5.  **Publishing**: The downsampled NumPy array (now shape (M, 4), where M <= N) is converted back into a new ``sensor_msgs/msg/PointCloud2`` message.

    * This output message always has the fixed, 16-byte layout: ``[x(float32), y(float32), z(float32), intensity(float32)]``.
    * The ``header.frame_id`` and ``header.stamp`` are copied from the input message to maintain spatial and temporal consistency.
    * If the resulting cloud is empty after filtering, nothing is published.

ROS 2 Interface
---------------

**Parameters:**

.. list-table:: Voxel Downsampler Parameters
   :header-rows: 1
   :widths: 30 50 70

   * - **Parameter**
     - **Value (from params.yaml)**
     - **Meaning**
   * - ``voxel_size``
     - ``0.2``
     - The edge length (in meters) of a single voxel cube.
   * - ``min_points_per_voxel``
     - ``1``
     - The minimum number of points a voxel must contain to be included in the output.
   * - ``filter_ground``
     - ``false``
     - If True, enables the simple ``z``-based ground removal filter.
   * - ``ground_threshold``
     - ``-1.5``
     - The ``z`` height (in meters) to use for ground filtering. Points at or below this height are removed.
   * - ``input_topic``
     - ``"/kitti/pointcloud_raw"``
     - The ROS 2 topic to subscribe to for incoming ``PointCloud2`` messages.
   * - ``output_topic``
     - ``"preprocessing/downsampled_cloud"``
     - The ROS 2 topic to publish the filtered and downsampled ``PointCloud2`` messages on.
   * - ``output_qos_reliability``
     - ``"reliable"`` (Script Default)
     - The QoS reliability policy for the publisher. This parameter was not specified in ``params.yaml``, so it uses the script's default.

**Subscriptions:**

* ``<input_topic>`` (``sensor_msgs/msg/PointCloud2``):
    * Subscribes using ``qos_profile_sensor_data`` (typically BEST_EFFORT).

**Publications:**

* ``<output_topic>`` (``sensor_msgs/msg/PointCloud2``):
    * Publishes using a configurable QoS (defaults to RELIABLE for easier debugging with tools like ``ros2 topic echo``).
    * The output format is always 16-byte points: [x, y, z, intensity], all ``FLOAT32``.


**Launch Files:**

`preprocessing.launch.py` 
~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** `src/lidar_preprocessing/launch/preprocessing.launch.py`

This launch file starts the voxel downsampling node. 

.. literalinclude:: ../../../src/lidar_preprocessing/launch/preprocessing.launch.py
   :language: python
   :caption: lidar_preprocessing/launch/preprocessing.launch.py
   :linenos:


Compile and Run
-----------------


**1. Build the Workspace**

Build the workspace (if not built already):

.. code-block:: bash

   colcon build --symlink-install
   source install/setup.bash

**2. Run and Visualize**

You will need two sourced ROS 2 terminals and the Foxglove Studio application. 

**Terminal 1: Start Foxglove Bridge**
Launch the websocket bridge that Foxglove Studio uses to communicate with ROS 2.

.. code-block:: bash

   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

**Foxglove Studio GUI**

1. Open Foxglove Studio.
2. Navigate to **Open Connection** -> **Foxglove WebSocket**.
3. Ensure the URL is ``ws://localhost:8765`` and click **Open**.
4. Load the pre-configured layout: **Layouts** -> **Personal** -> **Add**, and navigate to your workspace to select ``lidar_preprocessing/config/lidar_preprocessing_layout.json``.

**Terminal 2: Launch the KITTI Publisher**
Once the bridge and Foxglove are running and connected, launch both the ``kitti_data_loader` and the ``lidar_preprocessing`` nodes:

.. code-block:: bash

   ros2 launch lidar_preprocessing preprocessing.launch.py


.. Parameters
.. -----------

.. This page shows the YAML configuration files used by the package
.. ``lidar_preprocessing`` along with short explanations for important keys.

.. .. literalinclude:: ../../../src/lidar_preprocessing/config/params.yaml
..    :language: yaml
..    :caption: lidar_preprocessing/config/params.yaml
..    :linenos:

.. Description
.. ~~~~~~~~~~~

.. +------------------------+---------+-----------------------------------------------+
.. | Parameter              | Default | Meaning                                       |
.. +========================+=========+===============================================+
.. | voxel_size             | 0.2     | Cubic voxel edge length in meters             |
.. +------------------------+---------+-----------------------------------------------+
.. | min_points_per_voxel   | 1       | Minimum points required to keep a voxel       |
.. +------------------------+---------+-----------------------------------------------+
.. | filter_ground          | false   | If true, drop points below ground_threshold   |
.. +------------------------+---------+-----------------------------------------------+
.. | ground_threshold       | -1.2    | Z threshold used when filter_ground is true   |
.. +------------------------+---------+-----------------------------------------------+


.. If you have more than one YAML in this package, repeat:
..  .. literalinclude:: ../../../src/lidar_preprocessing/config/another.yaml
..     :language: yaml
..     :caption: lidar_preprocessing/config/another.yaml
..     :linenos: