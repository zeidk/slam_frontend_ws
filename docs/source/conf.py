# -- Path setup --------------------------------------------------------------
import os
import sys
from pathlib import Path

# Get the workspace root from the current file's path (<ws>/docs/source/conf.py)
WS_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS_ROOT / "src"))
for pkg in ("kitti_data_loader", "lidar_preprocessing"):
    sys.path.insert(0, str(WS_ROOT / "src" / pkg))

# # -- Path setup --------------------------------------------------------------
# import os
# import sys
# from pathlib import Path

# sys.path.insert(0, os.path.abspath('./_ext'))
# sys.path.insert(0, os.path.abspath('../../launch'))
# WS_ROOT = Path(__file__).resolve().parents[2]  # <ws>/docs/source -> <ws>

# # Add BOTH the workspace src/ (for flat cases) AND each nested package directory:
# sys.path.insert(0, str(WS_ROOT / "src"))  # harmless + useful if any flat pkgs exist
# for pkg in ("kitti_data_loader", "lidar_preprocessing"):
#     sys.path.insert(0, str(WS_ROOT / "src" / pkg))  # required for nested layout

# -- Project info ------------------------------------------------------------
project = "ROS2 SLAM Frontend"
copyright = "2025, zeidk"
author = "zeidk"
release = "0.0.1"

# -- Extensions --------------------------------------------------------------
extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinxcontrib.mermaid",  # needed for .. mermaid:: in pipeline.rst
    "sphinx_design",
    "sphinx_autodoc_typehints",
    "sphinx_copybutton"
]
autosummary_generate = True

# Mock ROS deps so docs build even without ROS installed in the venv
autodoc_mock_imports = [
    "rclpy",
    "sensor_msgs",
    "sensor_msgs_py",
    "std_msgs",
    "nav_msgs",
    "geometry_msgs",
    # "numpy",
    "python3-numpy",
    "yaml_doc",  # Add your custom extension
    "ament_index_python",
    "launch",
    "launch_ros",
    "builtin_interfaces",
    "tf2_ros",
    "scipy",
    "sensor_msgs.msg",
    "sensor_msgs_py.point_cloud2",
    "std_msgs.msg",
    "builtin_interfaces.msg",
    "rclpy.qos",
    "transforms3d",
    "tf_transformations"
]

autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'private-members': True,   # <— include _private names
    'show-inheritance': True,
}


# Pure RST (remove .md if you aren’t mixing Markdown)
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
html_theme = "sphinx_rtd_theme"
# html_theme = "furo"
