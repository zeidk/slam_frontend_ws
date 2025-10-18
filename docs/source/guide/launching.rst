.. _launching-guide:

Launching Nodes
===============

This section describes the launch files used to start the nodes in this workspace.

`kitti_data_loader` Launch File
---------------------------------

**File:** `src/kitti_data_loader/launch/kitti_data_loader.launch.py`

This launch file starts the KITTI publisher node with parameters loaded from
the configuration file. It also allows all parameters to be overridden from
the command line. 

.. literalinclude:: ../../../src/kitti_data_loader/launch/kitti_data_loader.launch.py
   :language: python
   :caption: kitti_data_loader/launch/kitti_data_loader.launch.py
   :linenos:


`lidar_preprocessing` Launch File
---------------------------------

**File:** `src/lidar_preprocessing/launch/preprocessing.launch.py`

This launch file starts the voxel downsampling node. 

.. literalinclude:: ../../../src/lidar_preprocessing/launch/preprocessing.launch.py
   :language: python
   :caption: lidar_preprocessing/launch/preprocessing.launch.py
   :linenos:
