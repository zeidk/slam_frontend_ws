from setuptools import setup
import os
from glob import glob

package_name = "lidar_preprocessing"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "pandas"],
    zip_safe=True,
    maintainer="zeid",
    maintainer_email="zeidk@umd.edu",
    description="LiDAR point cloud preprocessing (downsampling, filtering)",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "voxel_downsampler = lidar_preprocessing.voxel_downsampler:main"
        ],
    },
)
