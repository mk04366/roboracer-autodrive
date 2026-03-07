# Overview

The `system_launch` directory contains **ROS 2 launch files** responsible for orchestrating the complete **RoboRacer / AutoDRIVE autonomous driving stack**.  
These launch files coordinate the startup, configuration, and interconnection of multiple ROS 2 nodes, enabling seamless deployment of the full autonomy pipeline.

This module serves as the **entry point** for running the system in simulation or experimental setups.

---

## Purpose

The primary goals of the `system_launch` module are:

- To provide a **centralized bring-up mechanism** for the entire autonomy stack
- To ensure correct startup order and parameter configuration
- To simplify system execution for users and developers
- To support multiple operating modes (e.g., simulation, visualization, debugging)

By encapsulating launch logic in one place, system complexity is significantly reduced.

---

## Functionality

The launch files in this directory typically perform the following tasks:

- Start the AutoDRIVE simulator interface
- Launch localization, planning, and control nodes
- Load configuration parameters (YAML files)
- Handle topic remapping and namespace management
- Enable optional visualization and debugging tools

These launch files allow the autonomy stack to be executed with a **single command**.

---

## Integration in the Autonomy Stack

`system_launch` acts as the **top-level coordinator** for the following modules:

- `autodrive_f1tenth` – simulator and vehicle interface
- `localization` – vehicle state estimation
- `global-planning` – trajectory generation
- `control` or `control_grampc` – control execution
- `ros-foxglove-bridge` – optional visualization

This ensures all components operate within a shared ROS 2 environment.

---

## Directory Structure

A typical structure of the `system_launch` directory is:

```text
system_launch/
├── launch/
│   ├── system_bringup.launch.py
│   ├── simulation.launch.py
│   └── visualization.launch.py
├── config/
│   └── system_params.yaml
├── package.xml
└── CMakeLists.txt
```
