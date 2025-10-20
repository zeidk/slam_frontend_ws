
LOAM Feature Extraction
======================================

Module
-------

.. automodule:: loam_feature_extraction.feature_extractor
   :members:
   :undoc-members:
   :show-inheritance:


Launch Files
-------------

`feature_extraction.launch.py` 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** `src/loam_feature_extraction/launch/feature_extraction.launch.py`

This launch file starts the voxel downsampling node. 

.. literalinclude:: ../../../src/loam_feature_extraction/launch/feature_extraction.launch.py
   :language: python
   :caption: loam_feature_extraction/launch/feature_extraction.launch.py
   :linenos:



Parameters
-----------

This page shows the YAML configuration file used by the package
``loam_feature_extraction`` along with short explanations for key parameters.

.. literalinclude:: ../../../src/loam_feature_extraction/config/params.yaml
   :language: yaml
   :caption: loam_feature_extraction/config/params.yaml
   :linenos:

Description
~~~~~~~~~~~

.. list-table:: LOAM Feature Extraction Parameters
   :header-rows: 1
   :widths: 20 28 52

   * - **Parameter**
     - **Default**
     - **Meaning**
   * - ``num_rings``
     - 16
     - Number of LiDAR laser beams (for example, VLP-16 has 16).
   * - ``neighbor_size``
     - 5
     - Number of neighboring points used to compute curvature. Total neighbors = 2 × *m* (lecture notation).
   * - ``edge_percentage``
     - 2.0
     - Percentage of points with highest curvature classified as edge features.
   * - ``planar_percentage``
     - 2.0
     - Percentage of points with lowest curvature classified as planar features.
   * - ``min_range``
     - 1.0
     - Minimum valid range for LiDAR points (meters).
   * - ``max_range``
     - 100.0
     - Maximum valid range for LiDAR points (meters).
   * - ``input_topic``
     - ``"preprocessing/downsampled_cloud"``
     - Input topic providing the filtered and downsampled point cloud.
   * - ``edge_topic``
     - ``"features/edge_cloud"``
     - Output topic for detected edge features.
   * - ``planar_topic``
     - ``"features/planar_cloud"``
     - Output topic for detected planar features.




.. If you have more than one YAML in this package, repeat:
..  .. literalinclude:: ../../../src/lidar_preprocessing/config/another.yaml
..     :language: yaml
..     :caption: lidar_preprocessing/config/another.yaml
..     :linenos: