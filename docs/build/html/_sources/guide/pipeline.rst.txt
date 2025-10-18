SLAM Pipeline Overview
======================

The pipeline consists of three stages connected by ROS 2 topics.

Data Flow
---------

.. mermaid::

   flowchart LR
      A["/points_raw"] --> B[Preprocessing]
      B -->|"/points_filtered"| C[Feature Extraction]
      C -->|"/keypoints/corners"| C
      C -->|"/keypoints/planes"| C
      B -->|"/points_filtered"| D[Scan Matching ICP]
      D -->|"/points_aligned"| D
      D -->|"/odom"| E(("Odometry"))

Stage 1: Preprocessing
----------------------

**Package:** ``slam_preprocessing``

**Input:** ``/points_raw``  
**Output:** ``/points_filtered``

Operations:

- Z-axis pass-through filtering.
- Voxel grid down-sampling.

Stage 2: Feature Extraction
---------------------------

**Package:** ``slam_feature_extraction``

**Input:** ``/points_filtered``  
**Outputs:** ``/keypoints/corners``, ``/keypoints/planes``

Operations:

- Curvature proxy estimation.
- Separation into high-curvature (corners) and low-curvature (planes).

Stage 3: Scan Matching
----------------------

**Package:** ``slam_scan_matching``

**Input:** ``/points_filtered``  
**Outputs:** ``/points_aligned``, ``/odom``

Operations:

- Iterative Closest Point (ICP) between consecutive scans.
- Odometry integration.

Run the Entire Pipeline
-----------------------

.. code-block:: bash

   ros2 launch slam_pipeline demo_pipeline.launch.py
