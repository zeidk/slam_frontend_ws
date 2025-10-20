=======================
KITTI Dataset Overview
=======================

The **KITTI Odometry Benchmark** is a standard dataset used for evaluating visual and LiDAR odometry and SLAM algorithms.  
The dataset consists of several components providing calibration, pose, and LiDAR scan data.  
This section describes the contents of each compressed archive used in this project.


------------------------------------
1. data_odometry_calib.zip
------------------------------------

**Description**

Contains sensor calibration files for each KITTI odometry sequence.  
These files define the rigid-body transformations and projection parameters between the different sensors onboard the vehicle.

**Folder Structure**

::

    data_odometry_calib/
    ├── calib_00.txt
    ├── calib_01.txt
    ├── ...
    └── calib_21.txt

**File Format**

Each file corresponds to one sequence and contains entries such as:

- ``P0-P3``: Projection matrices for the four stereo cameras.  
- ``Tr_velo_to_cam``: Transformation matrix from the LiDAR (Velodyne) to the camera coordinate system.  
- ``Tr_imu_to_velo``: Transformation matrix from the IMU to the LiDAR coordinate system.

**Usage**

Calibration files are essential for transforming point clouds, images, and poses into a consistent coordinate frame.  
They are loaded by preprocessing or registration nodes to ensure geometric alignment between sensors.

---------------------------------
2. data_odometry_poses.zip
---------------------------------

**Description**

Contains the ground-truth poses for the odometry benchmark sequences.

**Folder Structure**

::

    data_odometry_poses/
    ├── dataset/
    │   └── poses/
    │       ├── 00.txt
    │       ├── 01.txt
    │       ├── ...
    │       └── 10.txt

**File Format**

Each ``.txt`` file corresponds to a sequence and contains one line per frame, describing the **3×4** transformation matrix
representing the vehicle’s pose in the world coordinate frame:

.. code-block:: text

    r11 r12 r13 tx  r21 r22 r23 ty  r31 r32 r33 tz

where ``r_ij`` form the rotation matrix and ``tx, ty, tz`` define translation.

**Usage**

These poses are typically used for benchmarking odometry or SLAM systems against ground truth.  
They can also be used to reconstruct trajectories for visualization in Foxglove Studio or RViz.

---------------------------------
3. data_odometry_velodyne.zip
---------------------------------

**Description**

Contains raw LiDAR scans from a Velodyne HDL-64E sensor collected during the KITTI odometry sequences.

**Folder Structure**

::

    data_odometry_velodyne/
    ├── dataset/
    │   └── sequences/
    │       ├── 00/
    │       │   └── velodyne/
    │       │       ├── 000000.bin
    │       │       ├── 000001.bin
    │       │       └── ...
    │       ├── 01/
    │       └── ...
    │       └── 21/

**File Format**

Each ``.bin`` file stores a single LiDAR frame as a sequence of floats:  
(x, y, z, reflectance), each taking **4×4 bytes = 16 bytes per point**.

The files can be read using:

.. code-block:: python

    import numpy as np
    scan = np.fromfile('000000.bin', dtype=np.float32).reshape(-1, 4)

**Usage**

The LiDAR data form the basis for odometry and mapping experiments.
They are typically loaded by nodes such as ``kitti_data_loader`` or preprocessing modules for voxelization,
feature extraction, and registration.

---------------------------------
4. Integration Notes
---------------------------------

- Ensure that all three archives are extracted into the same directory (e.g. ``~/datasets/kitti``).
- The folder hierarchy should look like:

::

    ~/datasets/kitti/
    ├── data_odometry_calib/
    ├── data_odometry_poses/
    └── data_odometry_velodyne/

- These datasets are referenced by ROS 2 launch files via parameters such as ``kitti_data_dir``.
- For efficient access, store the dataset on an SSD and avoid decompressing directly into network drives.

---------------------------------
5. References
---------------------------------

- KITTI Vision Benchmark Suite: https://www.cvlibs.net/datasets/kitti/
- Geiger, A., Lenz, P., Stiller, C., & Urtasun, R. (2013). *Vision meets Robotics: The KITTI Dataset.*  
  International Journal of Robotics Research (IJRR).
