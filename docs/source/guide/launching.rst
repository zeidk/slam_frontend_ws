Launching the System
====================

The KITTI data loader uses a ROS 2 launch file to start the publisher node
with parameters loaded from a YAML configuration file.

Launch File
-----------

.. literalinclude:: ../../../src/kitti_data_loader/launch/kitti_data_loader.launch.py
   :language: python
   :caption: launch/kitti_data_loader.launch.py
   :linenos:
   :emphasize-lines: 8-9,11-18

Usage
-----

Basic Launch
~~~~~~~~~~~~

Start the KITTI publisher with default parameters:

.. code-block:: bash

   ros2 launch kitti_data_loader kitti_data_loader.launch.py

The node will automatically:

- Load parameters from ``config/params.yaml``
- Begin publishing point clouds and poses at 10 Hz
- Publish on topics ``/kitti/pointcloud_raw`` and ``/kitti/ground_truth_pose``

What the Launch File Does
--------------------------

The launch file performs three main steps:

1. **Locates the package**: Uses ``get_package_share_directory()`` to find where
   the package is installed
2. **Loads configuration**: Constructs the path to ``config/params.yaml`` in the
   package's share directory
3. **Starts the node**: Launches ``kitti_publisher`` with the loaded parameters

Nodes Started
-------------

kitti_publisher_node
~~~~~~~~~~~~~~~~~~~~

:Package: ``kitti_data_loader``
:Executable: ``kitti_publisher``
:Output: ``screen`` (logs printed to terminal)
:Parameters: Loaded from ``config/params.yaml``

See :class:`kitti_data_loader.kitti_publisher.KittiPublisher` for detailed
node documentation.

Configuration
-------------

All runtime parameters are defined in the YAML configuration file. See
:doc:`/guide/configuration` for the complete parameter reference.

To customize behavior, edit ``config/params.yaml`` in your workspace before
building the package:

.. code-block:: bash

   # Edit the configuration
   nano ~/github/slam_frontend_ws/src/kitti_data_loader/config/params.yaml
   
   # Rebuild and install
   cd ~/github/slam_frontend_ws
   colcon build --packages-select kitti_data_loader
   source install/setup.bash
   
   # Launch with new configuration
   ros2 launch kitti_data_loader kitti_data_loader.launch.py

Topics Published
----------------

When the launch file runs, the following topics become available:

.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * - Topic
     - Message Type
     - Description
   * - ``/kitti/pointcloud_raw``
     - ``sensor_msgs/PointCloud2``
     - Velodyne LiDAR point clouds (x, y, z, intensity)
   * - ``/kitti/ground_truth_pose``
     - ``geometry_msgs/PoseStamped``
     - Ground-truth vehicle poses from KITTI

TF Frames
---------

The node also broadcasts a TF transform:

``odom`` → ``velodyne``
   Transform from the odometry frame to the LiDAR sensor frame, derived from
   ground-truth poses.

Troubleshooting
---------------

Package not found
~~~~~~~~~~~~~~~~~

.. code-block:: text

   Package 'kitti_data_loader' not found

**Solution**: Ensure you've sourced your workspace:

.. code-block:: bash

   source ~/github/slam_frontend_ws/install/setup.bash

Configuration file not found
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: text

   [ERROR] [launch]: Caught exception in launch (see debug for traceback): ...

**Solution**: The package installation may be incomplete. Rebuild:

.. code-block:: bash

   cd ~/github/slam_frontend_ws
   colcon build --packages-select kitti_data_loader --symlink-install

Node exits immediately
~~~~~~~~~~~~~~~~~~~~~~

Check that:

1. The KITTI dataset path in ``params.yaml`` is correct
2. The dataset sequence exists (e.g., ``00``)
3. Required files are present:

   - ``data_odometry_velodyne/dataset/sequences/00/velodyne/*.bin``
   - ``data_odometry_poses/dataset/poses/00.txt``

View the node's log output for specific error messages.

See Also
--------

- :doc:`/guide/configuration` - Parameter configuration reference
- :doc:`/guide/install_run` - Installation and setup guide  
- :class:`kitti_data_loader.kitti_publisher.KittiPublisher` - Node API reference