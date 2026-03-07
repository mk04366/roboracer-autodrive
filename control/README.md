# Overview

The control module implements classical and lightweight control algorithms for autonomous racing within the AutoDRIVE / RoboRacer ROS 2 ecosystem.
It is responsible for converting high-level planning information and vehicle state estimates into low-level steering and throttle commands.

Unlike control_grampc, which focuses on optimization-based control, this module emphasizes deterministic, computationally efficient controllers suitable for real-time execution and rapid testing.

## Purpose

The main goals of the control module are:

- To provide baseline controllers for autonomous driving

- To enable real-time control with minimal computational overhead

- To serve as a comparison point for optimal and learning-based controllers

- To offer stable and interpretable control behavior

These controllers are especially useful during early development, debugging, and low-speed operation.

## Control Strategies

The control module typically includes one or more of the following approaches:

### Path Tracking Controllers

- Pure Pursuit

- Stanley Controller

- Geometric tracking methods

These controllers compute steering commands based on geometric relationships between the vehicle pose and a reference path.

### Longitudinal Control

Proportional (P) or PID-based speed controllers

Acceleration or throttle regulation based on target velocity

### Reactive Control

Rule-based control logic

Safety overrides (e.g., speed reduction in sharp turns)

## Integration in the Autonomy Stack

The control module interacts with the rest of the system as follows:

- Inputs
  - Vehicle pose and velocity from localization

  - Reference trajectory or waypoints from planning modules

- Outputs
  - Steering command

  - Throttle or speed command

- Communication
  - ROS 2 topics using standard or custom message types

The module is designed to be modular, allowing individual controllers to be swapped or extended easily.

## Directory Structure

A typical layout of the control directory is:

```
control/
├── src/                # Controller implementations
├── include/            # Controller interfaces and utility headers
├── config/             # Controller parameters and tuning files
├── launch/             # ROS 2 launch files
├── CMakeLists.txt
└── package.xml
```

## Configuration and Tuning

Controller behavior is governed by tunable parameters such as:

- Lookahead distance (for Pure Pursuit)

- Gain values (for PID or Stanley controllers)

- Maximum steering angle and speed limits

These parameters are typically provided via:

- YAML configuration files

- ROS 2 parameter server

Proper tuning is essential for stable tracking performance, especially at higher speeds.

## Usage

The controllers in this module can be launched independently or as part of the full system bring-up.

```
ros2 launch control ${controller_name}.launch.py
```

Following nodes are already present in the `full_system_launch_foxglove.py` file and can be run from there alongside other required nodes.

Ensure that:

- Localization and planning nodes are active

- Required topics are correctly connected

- Vehicle or simulator interfaces are running
