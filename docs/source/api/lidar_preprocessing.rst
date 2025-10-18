
KITTI LiDAR Preprocessing Module
======================================

.. automodule:: lidar_preprocessing.voxel_downsampler
   :members:
   :undoc-members:
   :show-inheritance:

KITTI LiDAR Preprocessing Configuration
========================================

This page shows the YAML configuration files used by the package
``lidar_preprocessing`` along with short explanations for important keys.

.. literalinclude:: ../../../src/lidar_preprocessing/config/params.yaml
   :language: yaml
   :caption: lidar_preprocessing/config/params.yaml
   :linenos:

Key Parameters
--------------

+------------------------+---------+-----------------------------------------------+
| Parameter              | Default | Meaning                                       |
+========================+=========+===============================================+
| voxel_size             | 0.2     | Cubic voxel edge length in meters             |
+------------------------+---------+-----------------------------------------------+
| min_points_per_voxel   | 1       | Minimum points required to keep a voxel       |
+------------------------+---------+-----------------------------------------------+
| filter_ground          | false   | If true, drop points below ground_threshold   |
+------------------------+---------+-----------------------------------------------+
| ground_threshold       | -1.2    | Z threshold used when filter_ground is true   |
+------------------------+---------+-----------------------------------------------+


# If you have more than one YAML in this package, repeat:
# .. literalinclude:: ../../../src/lidar_preprocessing/config/another.yaml
#    :language: yaml
#    :caption: lidar_preprocessing/config/another.yaml
#    :linenos: