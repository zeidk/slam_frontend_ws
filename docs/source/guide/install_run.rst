Installation
========================

This guide explains how to install prerequisites for ENPM818Z (Lecture 2C).

Prerequisites
-------------

- Ubuntu 22.04 or later
- ROS 2 (Humble, Iron, or Jazzy)
- Python 3.8+
- Colcon

.. note::

   The provided source code was developed and tested on Ubuntu 24.04 with ROS 2 Jazzy.

Install Foxglove Studio
------------------------

- Go to https://foxglove.dev/download
- Download the .deb file (typically x64)

.. code-block:: bash

   cd <.deb location folder>
   sudo dpkg -i <file>.deb

Clone the ROS Workspace
------------------------

.. code-block:: bash

   cd <choose path> # e.g., cd ~
   git clone https://github.com/zeidk/slam_frontend_ws.git
   cd ~/slam_frontend_ws
   rosdep install --from-paths src --ignore-src -r -y

Create a Virtual Environment for ROS 2
---------------------------------------

We will use a virtual environment to install Python packages with ``pip``. 
The bash function below (`download here <https://drive.google.com/file/d/1SIORtNEczLuycxScXvbgNowFZIG9tZLG/view?usp=sharing>`_) will create and activate a virtual environment.
 

.. code-block:: bash

   function slam_frontend_ws {
      # --- ROS 2 and Workspace Setup ---
      VENV="${HOME}/ros2_venv"
      WS="/home/zeidk/github/slam_frontend_ws" # Match this with your location
      ROS2_DISTRO="jazzy" # Or your primary distro

      # --- Handle Python virtual environment ---
      # Check if a venv is already active by checking the VIRTUAL_ENV variable
      if [[ -z "$VIRTUAL_ENV" ]]; then
         # Venv is not active, proceed with creation/activation
         if [ ! -d "${VENV}" ]; then
            echo "🐍 Creating virtual environment at ${VENV}..."
            python3 -m venv "${VENV}"
            echo "✅ Virtual environment created."
         fi

         echo "▶️ Activating virtual environment..."
         source "${VENV}/bin/activate"
         echo "✅ ROS 2 + venv ready."
      else
         # A venv is already active, check if it's the correct one
         if [[ "$VIRTUAL_ENV" != "$VENV" ]]; then
            echo "⚠️ Another virtual environment is already active: $VIRTUAL_ENV"
            echo "   Deactivate it first with 'deactivate' or proceed with caution."
         else
            echo "✅ Correct virtual environment is already active."
         fi
      fi

      # Source ROS 2
      if [ -f "/opt/ros/${ROS2_DISTRO}/setup.zsh" ]; then
         source "/opt/ros/${ROS2_DISTRO}/setup.zsh"
      fi

      # Source the workspace
      if [ -f "${WS}/install/setup.zsh" ]; then
         source "${WS}/install/setup.zsh"
      fi

      # Setup argcomplete after everything is sourced
      if command -v register-python-argcomplete >/dev/null; then
            eval "$(register-python-argcomplete ros2)"
            eval "$(register-python-argcomplete colcon)"
      fi

      cd "${WS}"
   }

   # call the function
   slam_frontend_ws



Install Dependencies
--------------------

Foxglove Bridge:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-$ROS_DISTRO-foxglove-bridge
   sudo apt-get install -y python3-pandas

Python tools used elsewhere in this project:

.. code-block:: bash

   python3 -m pip install --upgrade pip
   python3 -m pip install numpy scipy

Download KITTI Odometry Data
----------------------------

Obtain the following archives for sequence ``00`` from Google Drive:

.. raw:: html

   <p>
     <a href="https://drive.google.com/file/d/1iHmkDuLv0z0C5Svzf78GfInyy9XNG_jM/view?usp=sharing" target="_blank" rel="noopener noreferrer">data_odometry_calib.zip (200 kB)</a><br>
     <a href="https://drive.google.com/file/d/1jzhq24ORy2vv-V2mawcjVFvm47cJgBho/view?usp=sharinghttps://nist.gov" target="_blank" rel="noopener noreferrer">data_odometry_poses.zip (1.3 MB)</a><br>
     <a href="https://nist.gov" target="_blank" rel="noopener noreferrer">data_odometry_velodyne.zip</a>
   </p>

.. - <a href="https://www.google.com" target="_blank" rel="noopener noreferrer">data_odometry_velodyne.zip</a> (6.7 GB)
.. - `data_odometry_calib.zip <https://drive.google.com/file/d/1iHmkDuLv0z0C5Svzf78GfInyy9XNG_jM/view?usp=sharing>`_  (200 kB)
.. - `data_odometry_poses.zip <https://drive.google.com/file/d/1jzhq24ORy2vv-V2mawcjVFvm47cJgBho/view?usp=sharing>`_  (1.3 MB)

Unpack each .zip file in the same location. Example:

.. code-block:: text

   /home/zeidk/github/slam_frontend_ws/data/kitti/data_odometry_calib
   /home/zeidk/github/slam_frontend_ws/data/kitti/data_odometry_poses
   /home/zeidk/github/slam_frontend_ws/data/kitti/data_odometry_velodyne


