# Global Planning (raceline optimization)

This subpackage contains tools for generating lanes and optimal racelines
from a bitmap track image.  It is based on the TUM global raceline
optimization code and is intended to be used offline to preprocess maps
for the AutoDRIVE simulator and the roboracer project.

## Overview

* `lane_generator.py` – extract boundaries, lanes, centerline and
  opponent track points from a map image.
* `main_globaltraj.py` – compute minimum‑time trajectories (racelines)
  using the optimization routines.
* `reverse_trajectory.py` – utilities for reversing or manipulating
  generated trajectories.
* `requirements.txt` – Python dependencies for this submodule.

## Getting started

> **Note:** the steps below assume you are working inside
> `planner/global-planning`.  We recommend creating an isolated
> Python virtual environment (e.g. `venv` or Anaconda) because the
> raceline optimisation requires specific versions of `numpy` and
> `scikit-learn` that may conflict with other ROS packages.

1.  Change into the directory:

    ```sh
    cd ~/ros2_ws/src/roboracer-autodrive/planner/global-planning
    ```

2.  Create and activate a virtual environment (example using `venv`):

    ```sh
    sudo apt install python3.8-venv          # if not already installed
    python3 -m venv ./venv                   # create venv
    source ./venv/bin/activate              # activate it
    touch ./COLCON_IGNORE                   # exclude from ROS build
    ```

3.  Install the Python dependencies:

    ```sh
    pip install -r requirements.txt
    ```

    The requirements include `opencv-contrib-python` which provides the
    `ximgproc` module used by `lane_generator.py`.

## Processing a map

1.  Copy your map files (`.pgm`, `.png`, etc.) and corresponding
    configuration `.yaml` file into the `maps/` subdirectory.

2.  Edit `config/params.yaml` and set `map_name`, `map_img_ext`,
    `num_lanes`, `clockwise`, and safety distances to match your track.

3.  Run the populating script (if available) to move files into the
    expected structure.  For example:

    ```sh
    ./scripts/populate.sh
    ```

4.  Execute the lane generator to produce the centerline, boundaries and
    lane data.  A window will pop up showing intermediate results –
    press any key (or `q` to abort) to continue.

    ```sh
    python3 lane_generator.py
    ```

5.  Once lanes have been generated the data is saved under
    `outputs/<map_name>/` as CSV and NumPy arrays.  Typical filenames are
    `centerline.csv`, `lane_0.csv`, `inner_bound.csv`, etc.

## Computing optimal racelines

1.  With the lanes generated, run the global trajectory optimizer:

    ```sh
    python3 main_globaltraj.py
    ```

2.  The script will write raceline files (`traj_race_cl.csv`,
    `traj_race_cl_high_sampled.csv`, etc.) into the same outputs
    directory.

