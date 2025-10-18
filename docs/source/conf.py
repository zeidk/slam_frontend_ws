# -- Path setup --------------------------------------------------------------
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.abspath('./_ext'))
sys.path.insert(0, os.path.abspath('../../launch'))
WS_ROOT = Path(__file__).resolve().parents[2]  # <ws>/docs/source -> <ws>

# Add BOTH the workspace src/ (for flat cases) AND each nested package directory:
sys.path.insert(0, str(WS_ROOT / "src"))  # harmless + useful if any flat pkgs exist
for pkg in ("kitti_data_loader"):
    sys.path.insert(0, str(WS_ROOT / "src" / pkg))  # required for nested layout

# -- Project info ------------------------------------------------------------    
project = 'ROS2 SLAM Frontend'
copyright = '2025, zeidk'
author = 'zeidk'
release = '0.0.1'

# -- Extensions --------------------------------------------------------------
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinxcontrib.mermaid",     # needed for .. mermaid:: in pipeline.rst
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
    "numpy",
    "python3-numpy",
    'yaml_doc',  # Add your custom extension
    'ament_index_python',
    'launch',
    'launch_ros',
]

# Pure RST (remove .md if you aren’t mixing Markdown)
source_suffix = {".rst": "restructuredtext"}

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
html_theme = "sphinx_rtd_theme"