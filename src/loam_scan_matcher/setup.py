from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'loam_scan_matcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'tf-transformations',
    ],
    zip_safe=True,
    maintainer='zeid',
    maintainer_email='zeidk@umd.edu',
    description='Frame-to-frame scan matching (point-to-plane ICP) for LOAM features (NumPy + SciPy).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'loam_scan_matcher = loam_scan_matcher.scan_matcher:main',
        ],
    },
)
