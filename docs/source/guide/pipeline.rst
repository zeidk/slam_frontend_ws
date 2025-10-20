===============================
SLAM Front-End Pipeline Overview
===============================

The SLAM front-end consists of four interconnected stages that process LiDAR data from acquisition through pose estimation. Each stage is a self-contained ROS 2 node with detailed configuration and comprehensive documentation.

Architecture
============

.. code-block:: text

    KITTI Dataset
         |
         v
    [1] kitti_publisher (Ground Truth & Raw LiDAR)
         |
         ├─> /kitti/pointcloud_raw
         ├─> /kitti/ground_truth_pose
         └─> /tf (map -> base_link)
         |
         v
    [2] voxel_downsampler (Preprocessing)
         |
         ├─> /preprocessing/downsampled_cloud
         |
         v
    [3] loam_feature_extractor (Feature Extraction)
         |
         ├─> /features/edge_cloud
         ├─> /features/planar_cloud
         |
         v
    [4] loam_scan_matcher (Odometry Estimation)
         |
         ├─> /scan_match/odometry
         ├─> /scan_match/trajectory
         └─> /tf (map -> base_link)


Stage 1: KITTI Data Loader
==========================

**Package:** ``kitti_data_loader``  
**Node:** ``kitti_publisher``

**Input:** KITTI Odometry dataset files (Velodyne ``.bin``, poses ``.txt``, calibration)

**Outputs:**
  - ``/kitti/pointcloud_raw`` (``PointCloud2``): Raw Velodyne LiDAR scans
  - ``/kitti/ground_truth_pose`` (``PoseStamped``): Ground-truth vehicle pose
  - ``/tf``: Dynamic transform ``map → base_link`` (ground truth)
  - ``/tf_static``: Static transform ``base_link → velodyne`` (calibration)

**Frame Convention:**
  Converts KITTI camera coordinates (x=right, y=down, z=forward) to ROS standard (x=forward, y=left, z=up).

**Key Features:**
  - Loads Velodyne point clouds and ground-truth poses from KITTI dataset
  - Publishes data at configurable rate for playback simulation
  - Handles coordinate frame transformations (KITTI → ROS REP-103)
  - Provides ground truth for evaluation and validation

For full details, see: :doc:`../api/kitti_data_loader`


Stage 2: LiDAR Preprocessing
============================

**Package:** ``lidar_preprocessing``  
**Node:** ``voxel_downsampler``

**Input:** ``/kitti/pointcloud_raw`` (``PointCloud2``)

**Output:** ``/preprocessing/downsampled_cloud`` (``PointCloud2``)

**Operations:**
  - Voxel-grid downsampling: Groups points into cubic cells and retains centroids
  - Optional ground filtering: Removes points below a configurable Z threshold
  - Robust parsing: Handles variable point cloud field layouts
  - Sanitization: Removes NaN and Inf values

**Key Parameters:**
  - ``voxel_size`` (0.2 m): Cell size for downsampling
  - ``min_points_per_voxel`` (1): Minimum points to retain a voxel
  - ``filter_ground`` (false): Enable/disable ground removal
  - ``ground_threshold`` (-1.5 m): Z-level for ground filtering

**Purpose:**
  Reduces point cloud density while preserving geometric structure. Speeds up downstream feature extraction and ICP registration.

For full details, see: :doc:`../api/lidar_preprocessing`


Stage 3: Feature Extraction
===========================

**Package:** ``loam_feature_extraction``  
**Node:** ``loam_feature_extractor``

**Input:** ``/preprocessing/downsampled_cloud`` (``PointCloud2``)

**Outputs:**
  - ``/features/edge_cloud`` (``PointCloud2``): High-curvature points
  - ``/features/planar_cloud`` (``PointCloud2``): Low-curvature points

**Operations:**

1. **Ring Organization:** Groups points by vertical angle (LiDAR scan rings)
2. **Curvature Computation:** For each point, computes local surface curvature:

   .. math::

      c_i = \frac{1}{|S| \, \lVert \mathbf{p}_i \rVert}
             \left\lVert \sum_{j \in S} (\mathbf{p}_i - \mathbf{p}_j) \right\rVert

3. **Feature Classification:** Selects top-N highest and lowest curvature points per ring

**Key Parameters:**
  - ``num_rings`` (16): Number of laser beams (e.g., VLP-16)
  - ``neighbor_size`` (2): Half-size of neighborhood for curvature (m=2 → 4 neighbors total)
  - ``edge_percentage`` (2.0%): Percentage of points classified as edges
  - ``planar_percentage`` (4.0%): Percentage of points classified as planar
  - ``min_range`` (1.0 m): Minimum valid range filter
  - ``max_range`` (100.0 m): Maximum valid range filter

**Output Format:**
  Both edge and planar clouds use ``[x, y, z, intensity]`` format, maintaining the input cloud's ``frame_id`` (``velodyne``).

**Purpose:**
  Extracts sparse geometric primitives (edges and planes) that are robust to noise and suitable for registration.

For full details, see: :doc:`../api/loam_feature_extraction`


Stage 4: Scan Matching & Odometry
=================================

**Package:** ``loam_scan_matcher``  
**Node:** ``loam_scan_matcher``

**Inputs:**
  - ``/features/edge_cloud`` (``PointCloud2``)
  - ``/features/planar_cloud`` (``PointCloud2``)

**Outputs:**
  - ``/scan_match/odometry`` (``Odometry``): Estimated pose in map frame
  - ``/scan_match/trajectory`` (``Path``): Accumulated trajectory
  - ``/tf``: Dynamic transform ``map → base_link`` (estimated)

**Algorithm:**

1. **Feature Fusion:** Combines edge and planar features into single point cloud
2. **Downsampling:** Applies voxel-grid downsampling for efficiency
3. **Normal Estimation:** Computes per-point surface normals using k-NN PCA
4. **Point-to-Plane ICP:** Iterative registration against previous frame:

   .. math::

      \mathbf{n}_i^T \left( R\mathbf{q}_i + \mathbf{t} - \mathbf{p}_i \right) \approx 0

5. **SE(3) Optimization:** Solves directly in Lie algebra using exponential map:

   .. math::

      \exp(\boldsymbol{\xi}) = \begin{pmatrix} R & V\mathbf{t} \\ 0 & 1 \end{pmatrix}

6. **Pose Integration:** Accumulates relative transformations into global pose:

   .. math::

      T_{wb} \gets T_{wb} \cdot T_{\text{curr}}

**Key Parameters:**
  - ``voxel_size`` (0.2 m): Downsampling for ICP
  - ``max_icp_iter`` (20): Maximum iterations per frame
  - ``icp_dist_thresh`` (1.0 m): Correspondence distance threshold
  - ``knn_normals`` (20): Neighbors for normal estimation

**Frame Hierarchy:**

.. code-block:: text

    map (global reference, ground truth)
      └── base_link (robot body, estimated)
            └── velodyne (LiDAR sensor)

The scan matcher publishes the estimated ``map → base_link`` transform, which updates at each frame as pose is incrementally estimated.

**Evaluation:**

Ground truth is provided by the KITTI publisher's ``map → base_link`` transform. Compare estimated and ground-truth poses to evaluate accuracy.

For full details, see: :doc:`../api/loam_scan_matching`


Data Flow & Topics
==================

.. list-table:: Topic Reference
   :header-rows: 1
   :widths: 35 20 45

   * - **Topic**
     - **Type**
     - **Purpose**
   * - ``/kitti/pointcloud_raw``
     - ``PointCloud2``
     - Raw LiDAR scans from KITTI dataset
   * - ``/kitti/ground_truth_pose``
     - ``PoseStamped``
     - Ground-truth vehicle pose for validation
   * - ``/preprocessing/downsampled_cloud``
     - ``PointCloud2``
     - Downsampled and filtered point cloud
   * - ``/features/edge_cloud``
     - ``PointCloud2``
     - High-curvature feature points
   * - ``/features/planar_cloud``
     - ``PointCloud2``
     - Low-curvature feature points
   * - ``/scan_match/odometry``
     - ``Odometry``
     - Estimated pose in map frame
   * - ``/scan_match/trajectory``
     - ``Path``
     - Accumulated trajectory of estimated poses
   * - ``/tf``
     - ``TFMessage``
     - Dynamic transforms (both ground truth and estimates)
   * - ``/tf_static``
     - ``TFMessage``
     - Static calibration transforms


Running the Complete Pipeline
=============================

**1. Build the Workspace**

.. code-block:: bash

   colcon build --symlink-install
   source install/setup.bash

**2. Start Foxglove Bridge (Terminal 1)**

.. code-block:: bash

   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

**3. Open Foxglove Studio**

- Navigate to **Open Connection** → **Foxglove WebSocket**
- Enter ``ws://localhost:8765``
- Load layout from ``loam_scan_matcher/config/scan_matching_layout.json``

**4. Launch Complete Pipeline (Terminal 2)**

.. code-block:: bash

   ros2 launch loam_scan_matcher scan_matching.launch.py

This single launch command orchestrates:
  1. KITTI data playback
  2. Voxel downsampling
  3. Feature extraction
  4. Scan matching & odometry

**5. Monitor Progress**

Check pose estimation:

.. code-block:: bash

   ros2 topic echo /scan_match/odometry --field pose.pose.position

Monitor feature extraction:

.. code-block:: bash

   ros2 topic echo /features/edge_cloud --field width
   ros2 topic echo /features/planar_cloud --field width

Verify TF tree:

.. code-block:: bash

   ros2 run tf2_tools view_frames


Configuration & Tuning
======================

Each stage has its own parameter file in ``src/<package>/config/params.yaml``.

**For Performance Tuning:**

1. **Reduce point density** (faster, less accurate):
   - Increase ``voxel_size`` in preprocessing (e.g., 0.3 m)
   - Increase ``voxel_size`` in scan matcher (e.g., 0.3 m)

2. **Improve accuracy** (slower, more detailed):
   - Decrease ``voxel_size`` in preprocessing (e.g., 0.1 m)
   - Increase ``knn_normals`` in scan matcher (e.g., 30)
   - Increase ``max_icp_iter`` (e.g., 30)

3. **Balance edge/planar** features:
   - Adjust ``edge_percentage`` and ``planar_percentage`` in feature extractor
   - Monitor with ``/Plot!feature_widths`` in Foxglove

4. **Robustness to motion**:
   - Increase ``icp_dist_thresh`` in scan matcher if correspondences are sparse
   - Adjust ``min_range`` and ``max_range`` to focus on reliable regions


Evaluation & Validation
=======================

The KITTI publisher provides ground-truth poses, enabling accuracy evaluation:

.. code-block:: bash

   # Compare ground truth vs. estimated
   ros2 topic echo /kitti/ground_truth_pose --field pose.pose.position
   ros2 topic echo /scan_match/odometry --field pose.pose.position

**Metrics:**
  - **Absolute Trajectory Error (ATE):** L2 distance between estimated and ground-truth poses
  - **Relative Pose Error (RPE):** Accuracy of incremental motion estimates
  - **Temporal Alignment:** Ensure timestamps are synchronized

Documentation
==============

Each stage has comprehensive documentation:

- :doc:`../api/kitti_data_loader` - Data loading and coordinate transformations
- :doc:`../api/lidar_preprocessing` - Voxel downsampling and filtering
- :doc:`../api/loam_feature_extraction` - Feature extraction using curvature
- :doc:`../api/loam_scan_matching` - ICP registration and pose estimation

References
==========

- Zhang, J., & Singh, S. (2014). LOAM: LiDAR Odometry and Mapping in Real-time.
- ROS Enhancement Proposals (REP-103): Standard Units of Measure and Coordinate Conventions
- KITTI Vision Benchmark Suite: http://www.cvlibs.net/datasets/kitti/