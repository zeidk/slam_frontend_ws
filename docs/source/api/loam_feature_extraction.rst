==================================
ROS 2 LOAM Feature Extraction Node
==================================

Overview
--------

This script implements a ROS 2 node, ``loam_feature_extractor``, that subscribes to a ``sensor_msgs/msg/PointCloud2`` topic (typically the output of a voxel downsampling node) and extracts geometric features — **edge** and **planar** points — following the method described in the LOAM (LiDAR Odometry and Mapping) algorithm by Zhang & Singh (2014).

The node provides feature-level information suitable for subsequent stages such as odometry, mapping, or registration. It has been adapted from ENPM818Z *L2C* lecture content and is designed for research and teaching applications within the NIST SLAM Front-End testbed.

Core Functionality
------------------

The node performs the following sequence of operations for each incoming LiDAR scan:

1.  **Point Cloud Conversion**  
    Converts an input ``PointCloud2`` message into a NumPy array of shape (N, 4) containing ``[x, y, z, intensity]``.

2.  **Ring Organization**  
    Groups points into scan rings (laser channels) based on vertical angles.  
    *This preserves LiDAR beam structure (for example, 16 rings for a VLP-16 sensor).*

3.  **Curvature Computation**  
    For each point, computes local curvature using neighboring points within the same ring according to the LOAM formulation:

    .. math::

       c_i = \frac{1}{|S| \, \lVert \mathbf{p}_i \rVert}
              \left\lVert \sum_{j \in S} (\mathbf{p}_i - \mathbf{p}_j) \right\rVert

    where :math:`S` is the neighborhood containing :math:`m` points on each side of :math:`\mathbf{p}_i`.

4.  **Feature Selection**  

    * Points with **highest** curvature values are classified as **edge features**.  
    * Points with **lowest** curvature values are classified as **planar features**.  
    * The percentages of each type are configurable via ``edge_percentage`` and ``planar_percentage``.

5.  **Publishing**  
    The node publishes two new point clouds:

    - ``features/edge_cloud`` — points with high curvature (edges)
    - ``features/planar_cloud`` — points with low curvature (planes)

    Both outputs use the same format as the input cloud: ``[x, y, z, intensity]``.

ROS 2 Interface
---------------

**Parameters:**

.. list-table:: LOAM Feature Extractor Parameters
   :header-rows: 1
   :widths: 30 50 70

   * - **Parameter**
     - **Value (from params.yaml)**
     - **Meaning**
   * - ``num_rings``
     - ``16``
     - Number of LiDAR laser beams (for example, VLP-16 has 16 rings).
   * - ``neighbor_size``
     - ``5``
     - Number of neighboring points used to compute curvature.  
       The total neighborhood size is ``2 × m``.
   * - ``edge_percentage``
     - ``2.0``
     - Percentage of points per ring classified as edge features.
   * - ``planar_percentage``
     - ``2.0``
     - Percentage of points per ring classified as planar features.
   * - ``min_range``
     - ``1.0``
     - Minimum valid range for LiDAR points (in meters).
   * - ``max_range``
     - ``100.0``
     - Maximum valid range for LiDAR points (in meters).
   * - ``input_topic``
     - ``"preprocessing/downsampled_cloud"``
     - Input topic providing the filtered and downsampled point cloud.
   * - ``edge_topic``
     - ``"features/edge_cloud"``
     - Output topic for detected edge features.
   * - ``planar_topic``
     - ``"features/planar_cloud"``
     - Output topic for detected planar features.

**Subscriptions:**

* ``<input_topic>`` (``sensor_msgs/msg/PointCloud2``):  
  Subscribes to the preprocessed and downsampled LiDAR scan.  
  Typically this comes from the ``voxel_downsampler`` node.

**Publications:**

* ``<edge_topic>`` (``sensor_msgs/msg/PointCloud2``):  
  Publishes extracted edge features.

* ``<planar_topic>`` (``sensor_msgs/msg/PointCloud2``):  
  Publishes extracted planar features.


**Launch Files:**

*File:* ``src/loam_feature_extraction/launch/feature_extraction.launch.py``

This launch file starts the complete LOAM feature extraction pipeline, including all dependent stages required for full functionality.

It launches the following sequence:

1. **`kitti_publisher`** — publishes KITTI dataset LiDAR and pose data.  
2. **`voxel_downsampler`** — applies voxel-grid downsampling and optional ground filtering.  
3. **`loam_feature_extractor`** — extracts edge and planar features from the preprocessed cloud.

.. literalinclude:: ../../../src/loam_feature_extraction/launch/feature_extraction.launch.py
   :language: python
   :caption: loam_feature_extraction/launch/feature_extraction.launch.py
   :linenos:


Compile and Run
-----------------

**1. Build the Workspace**

Build the workspace (if not already built):

.. code-block:: bash

   colcon build --symlink-install
   source install/setup.bash

**2. Run and Visualize**

You will need two sourced ROS 2 terminals and the Foxglove Studio application.

**Terminal 1 - Start Foxglove Bridge**

Launch the websocket bridge that Foxglove Studio uses to communicate with ROS 2:

.. code-block:: bash

   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

**Foxglove Studio GUI**

1. Open Foxglove Studio.  
2. Go to **Open Connection** → **Foxglove WebSocket**.  
3. Enter ``ws://localhost:8765`` and click **Open**.  
4. Load the pre-configured layout from  
   ``loam_feature_extraction/config/feature_extraction_layout.json``  
   to visualize the following topics:

   - ``/kitti/pointcloud_raw``  
   - ``/preprocessing/downsampled_cloud``  
   - ``/features/edge_cloud``  
   - ``/features/planar_cloud``

**Terminal 2 - Launch the Feature Extraction Pipeline**

Once Foxglove Studio is connected, launch all nodes:

.. code-block:: bash

   ros2 launch loam_feature_extraction feature_extraction.launch.py

Module
--------

.. automodule:: loam_feature_extraction.feature_extractor
   :members:
   :undoc-members:
   :show-inheritance: