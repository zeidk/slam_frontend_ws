==================================
ROS 2 LOAM Scan Matching Node
==================================

Overview
--------

This script implements a ROS 2 node, ``loam_scan_matcher``, that subscribes to edge and planar feature point clouds and performs incremental pose estimation using **point-to-plane Iterative Closest Point (ICP)** registration. The node integrates relative transformations into a global pose, continuously publishing odometry, trajectory, and frame transforms.

The algorithm follows the LOAM (LiDAR Odometry and Mapping) framework by Zhang & Singh (2014) and is designed for real-time incremental LiDAR odometry. It has been adapted from ENPM818Z *L2C* lecture content and is suitable for research and teaching applications within the NIST SLAM Front-End testbed.

Core Functionality
------------------

The node performs the following sequence of operations for each incoming frame:

1.  **Feature Fusion**  
    Combines edge features (high curvature) and planar features (low curvature) from the current scan into a single point cloud for registration.

2.  **Voxel Downsampling**  
    Reduces point density using voxel-grid hashing to improve ICP convergence speed and robustness.

3.  **Normal Estimation**  
    Computes per-point surface normals on the reference frame using k-NN PCA. Normals are oriented consistently (toward +Z by heuristic) for reliable point-to-plane residuals.

4.  **Point-to-Plane ICP Registration**  
    Performs iterative closest point matching against the previous frame using point-to-plane constraints:

    .. math::

       \mathbf{n}_i^T \left( R\mathbf{q}_i + \mathbf{t} - \mathbf{p}_i \right) \approx 0

    where :math:`\mathbf{p}_i` are reference points, :math:`\mathbf{q}_i` are source points, :math:`\mathbf{n}_i` are surface normals, and the goal is to estimate :math:`R` and :math:`\mathbf{t}`.

5.  **SE(3) Optimization**  
    Solves the registration problem directly in Lie algebra (se(3)) using the **exponential map**:

    .. math::

       \exp(\boldsymbol{\xi}) = \begin{pmatrix} R & V\mathbf{t} \\ 0 & 1 \end{pmatrix}

    where :math:`\boldsymbol{\xi} = [\boldsymbol{\omega}, \mathbf{t}]^T` is a 6D twist vector. This allows direct least-squares optimization without singularities.

6.  **Pose Integration**  
    Integrates the estimated relative transformation into the global world-to-base transformation:

    .. math::

       T_{wb} \gets T_{wb} \cdot T_{curr}

7.  **Publishing**  
    The node publishes:

    - **Odometry** (``scan_match/odometry``): Current pose in the map frame as a ``nav_msgs/msg/Odometry`` message.
    - **Trajectory** (``scan_match/trajectory``): Accumulated path as a ``nav_msgs/msg/Path`` message.
    - **TF Transform** (``map → base_link``): Broadcast pose as a geometry frame transform.

Coordinate Frame Hierarchy
---------------------------

The scan matcher uses a **single global frame hierarchy** rooted in the **map frame**:

.. code-block:: text

    map (global reference)
      └── base_link (robot body frame)
            └── velodyne (LiDAR sensor frame)

**Frame Semantics:**

* **map**: Global reference frame provided by the KITTI ground truth publisher. All poses and point clouds are ultimately expressed in this frame.

* **base_link**: Robot body frame. Updated by the scan matcher with the estimated pose ``T_wb`` (world-to-body transformation).

* **velodyne**: LiDAR sensor frame. Point clouds are published in this frame by the KITTI publisher and feature extractor.

**Why Clouds Appear in Both Frames (Foxglove Display):**

When visualizing point clouds in Foxglove Studio, the same cloud data can be displayed in multiple frames because:

1. Point clouds carry a ``frame_id`` (e.g., ``velodyne``) indicating their native frame.
2. Foxglove uses the TF tree to transform clouds into any requested frame.
3. Selecting different panels with different ``frameId`` settings will show the same cloud data transformed into those frames.

For example:
- A cloud native to ``velodyne`` displayed in the ``map`` frame is transformed as: ``velodyne → base_link → map``
- The same cloud displayed in the ``base_link`` frame is transformed as: ``velodyne → base_link``

If the ``map → base_link`` transform is static (pose not changing), the clouds will appear identical in both frames. If the transform updates with new pose estimates, the clouds will appear to move relative to the map frame grid.

**Note:** To verify that pose estimation is working, check:

.. code-block:: bash

    ros2 topic echo /scan_match/odometry --field pose.pose.position

Position (x, y, z) should change with each new scan if the sensor is moving.

ROS 2 Interface
---------------

**Parameters:**

.. list-table:: LOAM Scan Matcher Parameters
   :header-rows: 1
   :widths: 30 50 70

   * - **Parameter**
     - **Value (from params.yaml)**
     - **Meaning**
   * - ``edge_topic``
     - ``"features/edge_cloud"``
     - Input topic for edge feature clouds from feature extractor.
   * - ``planar_topic``
     - ``"features/planar_cloud"``
     - Input topic for planar feature clouds from feature extractor.
   * - ``voxel_size``
     - ``0.2``
     - Voxel grid downsampling cell size (meters). Smaller values preserve detail; larger values speed up ICP.
   * - ``max_icp_iter``
     - ``20``
     - Maximum number of ICP iterations per frame. Early stopping occurs if update norm < 1e-6.
   * - ``icp_dist_thresh``
     - ``1.0``
     - Distance threshold (meters) for considering point-to-point correspondences in ICP. Correspondences beyond this distance are rejected.
   * - ``knn_normals``
     - ``20``
     - Number of neighbors used for k-NN surface normal estimation via PCA.
   * - ``frame_map``
     - ``"map"``
     - Global reference frame ID (from KITTI ground truth).
   * - ``frame_base``
     - ``"base_link"``
     - Robot body frame ID. Updated by scan matcher pose estimates.
   * - ``odom_topic``
     - ``"scan_match/odometry"``
     - Output topic for publishing odometry messages.
   * - ``trajectory_topic``
     - ``"scan_match/trajectory"``
     - Output topic for publishing trajectory paths.

**Subscriptions:**

* ``<edge_topic>`` (``sensor_msgs/msg/PointCloud2``):  
  Edge feature clouds (high curvature points) from the feature extractor.

* ``<planar_topic>`` (``sensor_msgs/msg/PointCloud2``):  
  Planar feature clouds (low curvature points) from the feature extractor.

**Publications:**

* ``<odom_topic>`` (``nav_msgs/msg/Odometry``):  
  Odometry message with pose and frame information in the map frame.

* ``<trajectory_topic>`` (``nav_msgs/msg/Path``):  
  Accumulated trajectory of all poses since launch.

* **TF Transform** (``geometry_msgs/msg/TransformStamped``):  
  Broadcast transform from ``<frame_map>`` to ``<frame_base>``.


**Launch Files:**

*File:* ``src/loam_scan_matcher/launch/scan_matching.launch.py``

This launch file starts the complete LOAM scan matching pipeline, including all dependent stages required for full functionality.

It launches the following sequence:

1. **Feature Extraction Pipeline** (included)  
   - KITTI data loader  
   - Voxel downsampler  
   - LOAM feature extractor  

2. **LOAM Scan Matcher** (this file)  
   - Subscribes to edge and planar features  
   - Publishes odometry, trajectory, and TF transforms  

.. literalinclude:: ../../../src/loam_scan_matcher/launch/scan_matching.launch.py
   :language: python
   :caption: loam_scan_matcher/launch/scan_matching.launch.py
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
   ``loam_scan_matcher/config/scan_matching_layout.json``  
   to visualize the following topics:

   - ``/features/edge_cloud`` (point cloud, colorized by intensity)  
   - ``/features/planar_cloud`` (point cloud, colorized by intensity)  
   - ``/scan_match/trajectory`` (path visualization)  
   - ``/scan_match/odometry`` (pose and position plots)

5. The layout includes:

   - **3D View (main)**: Point clouds, trajectory path, grid in ``map`` frame, and robot axes at ``base_link``.  
   - **Transform Tree**: Visualization of the TF hierarchy (``map → base_link → velodyne``).  
   - **Topic Graph**: Node and topic connectivity.  
   - **Feature Widths Plot**: Edge and planar cloud sizes over time.  
   - **Position Plot**: X, Y, Z trajectory coordinates.

**Terminal 2 - Launch the Scan Matching Pipeline**

Once Foxglove Studio is connected, launch all nodes:

.. code-block:: bash

   ros2 launch loam_scan_matcher scan_matching.launch.py

**Monitor Pose Estimation**

To verify that the scan matcher is successfully estimating pose (not stalled):

.. code-block:: bash

   ros2 topic echo /scan_match/odometry --field pose.pose.position

Position values should change as the KITTI dataset plays through the sequence.

Troubleshooting
---------------

**Pose Not Updating (Position Constant)**

If ``/scan_match/odometry`` shows the same position repeatedly:

1. Check that edge and planar clouds have sufficient points:

   .. code-block:: bash

      ros2 topic echo /features/edge_cloud --field width
      ros2 topic echo /features/planar_cloud --field width

2. Verify ICP convergence by reducing ``icp_dist_thresh`` or increasing ``max_icp_iter`` in params.yaml.

3. Check the TF tree:

   .. code-block:: bash

      ros2 run tf2_tools view_frames

   The tree should show ``map → base_link``.

**TF Transform Errors in Foxglove**

If Foxglove reports "Missing transform", ensure:

1. The ``frame_map`` and ``frame_base`` parameters match your layout's frame IDs.
2. The TF broadcaster is running (check ``/tf`` topic):

   .. code-block:: bash

      ros2 topic list | grep tf

**Noisy or Jumping Trajectories**

If the estimated path jitters or jumps:

1. Increase ``voxel_size`` to reduce noise sensitivity.
2. Increase ``icp_dist_thresh`` to include more correspondences.
3. Check that the feature extractor is producing balanced edge/planar distributions (see feature width plots).

Module
--------

.. automodule:: loam_scan_matcher.scan_matcher
   :members:
   :undoc-members:
   :show-inheritance: