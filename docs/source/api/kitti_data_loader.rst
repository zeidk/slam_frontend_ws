KITTI Data Loader
=================

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
~~~~~~~~~~~~~~

+-------------------------+-----------------------------+-------------------------------------------------------------+
| Parameter               | Default                     | Meaning                                                     |
+=========================+=============================+=============================================================+
| kitti_data_dir          | <change it>                 | Absolute path to the KITTI dataset directory.               |
+-------------------------+-----------------------------+-------------------------------------------------------------+
| dataset_sequence        | s00                         | Identifier of the KITTI dataset sequence to publish.        |
+-------------------------+-----------------------------+-------------------------------------------------------------+
| publish_rate_hz         | 10.0                        | Publishing frequency of KITTI sensor data in hertz.         |
+-------------------------+-----------------------------+-------------------------------------------------------------+
| pointcloud_topic        | /kitti/pointcloud_raw       | Topic name on which raw LiDAR point clouds are published.   |
+-------------------------+-----------------------------+-------------------------------------------------------------+
| ground_truth_pose_topic | /kitti/ground_truth_pose    | Topic name for ground-truth vehicle poses from KITTI.       |
+-------------------------+-----------------------------+-------------------------------------------------------------+
| base_frame_id           | odom                        | Frame ID representing the vehicle base (odometry frame).    |
+-------------------------+-----------------------------+-------------------------------------------------------------+
| lidar_frame_id          | velodyne                    | Frame ID representing the LiDAR sensor reference frame.     |
+-------------------------+-----------------------------+-------------------------------------------------------------+

.. If you have more than one YAML in this package, repeat:
..  .. literalinclude:: ../../../src/lidar_preprocessing/config/another.yaml
..     :language: yaml
..     :caption: lidar_preprocessing/config/another.yaml
..     :linenos: