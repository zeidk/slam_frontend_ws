KITTI Data Loader Module
===============================

.. automodule:: kitti_data_loader.kitti_publisher
   :members:
   :undoc-members:
   :show-inheritance:

KITTI Data Loader Configuration
===============================

.. literalinclude:: ../../../src/kitti_data_loader/config/params.yaml
   :language: yaml
   :caption: kitti_data_loader/config/params.yaml
   :linenos:

Key Parameters
--------------

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
