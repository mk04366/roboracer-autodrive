# Overview

The control_grampc module implements model-based optimal control for autonomous racing using GRAMPC (Gradient-based Augmented Lagrangian Model Predictive Control).
It is responsible for generating optimal steering and throttle commands for the RoboRacer vehicle based on the current vehicle state and reference trajectories.

This module is designed to operate within the AutoDRIVE / RoboRacer ROS 2 ecosystem, receiving state estimates from localization and reference information from planning modules, and publishing low-level control commands to the simulator.

## GRAMPC-Based Model Predictive Control

GRAMPC is a lightweight, gradient-based MPC framework suitable for real-time applications. Within this module:

- A vehicle dynamics model is defined (e.g., kinematic or dynamic bicycle model)
- A cost function penalizes tracking error, control effort, and constraint violations

- Constraints may include:
  - Steering limits
  - Acceleration bounds
  - State limits (e.g., velocity)

At each control step:

- The current vehicle state is received

- An optimal control problem is formulated

- GRAMPC solves the optimization over a finite horizon

- The first control action is applied (receding horizon principle)

## Integration in the Autonomy Stack

The `control_grampc` module interacts with other system components as follows:

- Inputs
  - Vehicle state (pose, velocity, orientation)

  - Reference trajectory or target state

- Outputs
  - Steering command

  - Throttle (or acceleration) command

- Communication
  - Uses ROS 2 topics and custom messages defined in autodrive_msgs

This controller can be used as a drop-in replacement for simpler controllers found in `control`.

## Directory Structure

A typical structure inside `control_grampc` may include:

```text
control_grampc/
├── src/                # GRAMPC controller implementation
├── include/            # Header files and model definitions
├── config/             # MPC parameters and tuning files
├── launch/             # ROS 2 launch files
├── CMakeLists.txt
└── package.xml
```

## Configuration and Tuning

Key MPC parameters that typically require tuning include:

- Prediction horizon length

- Cost weights for:
  - Tracking error

  - Control effort

  - Control rate changes

- Constraint bounds

## Usage

The GRAMPC controller is launched as part of the full system bring-up.

```
ros2 launch control_grampc control_grampc.launch.py
```

Ensure that:

- The simulator or vehicle interface is running

- Localization and planning nodes are active

- Required topics are correctly remapped

## Related Modules

- `control` – other classical control approaches for comparison
- `global-planning` – provides reference trajectories
- `localization` – provides vehicle state estimates
