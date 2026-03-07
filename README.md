# Introduction

The idea of this repository is to develop a modular and scalable simulation framework repository for the F1TENTH platform that integrates various autonomous driving algorithms across perception, planning, and control. The repository will enable future contributors to easily test and evaluate different algorithmic combinations under varying conditions, promoting a better way of experimentation, benchmarking, and rapid prototyping in a reproducible environment.

# Demonstration

[![FTG Algorithm Demo](https://img.youtube.com/vi/ZMOKa8dJni0/hqdefault.jpg)](https://www.youtube.com/watch?v=ZMOKa8dJni0)

# ROS 2 API

<p align="justify">
This directory hosts ROS 2 API (a meta-package), which supports modular algorithm development targetted towards autonomous driving. It uses client libraries for Python and C++, and can be therefore exploited by the users to develop their algorithms flexibly.
</p>

## DEPENDENCIES

[AutoDRIVE Devkit's ROS 2 API](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_ros2) has the following dependencies (tested with Python 3.8.10):

- Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator) and [AutoDRIVE Devkit](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit) (version sensitive):

  | Package          | Tested Version |
  | ---------------- | -------------- |
  | eventlet         | 0.33.3         |
  | Flask            | 1.1.1          |
  | Flask-SocketIO   | 4.1.0          |
  | python-socketio  | 4.2.0          |
  | python-engineio  | 3.13.0         |
  | greenlet         | 1.0.0          |
  | gevent           | 21.1.2         |
  | gevent-websocket | 0.10.1         |
  | Jinja2           | 3.0.3          |
  | itsdangerous     | 2.0.1          |
  | werkzeug         | 2.0.3          |

  ```bash
  $ pip3 install eventlet==0.33.3
  $ pip3 install Flask==1.1.1
  $ pip3 install Flask-SocketIO==4.1.0
  $ pip3 install python-socketio==4.2.0
  $ pip3 install python-engineio==3.13.0
  $ pip3 install greenlet==1.0.0
  $ pip3 install gevent==21.1.2
  $ pip3 install gevent-websocket==0.10.1
  $ pip3 install Jinja2==3.0.3
  $ pip3 install itsdangerous==2.0.1
  $ pip3 install werkzeug==2.0.3
  ```

- Generic dependencies for data processing and visualization (usually any version will do the job):

  | Package               | Tested Version |
  | --------------------- | -------------- |
  | attrdict              | 2.0.1          |
  | numpy                 | 1.13.3         |
  | pillow                | 5.1.0          |
  | opencv-contrib-python | 4.5.1.48       |

  ```bash
  $ pip3 install attrdict
  $ pip3 install numpy
  $ pip3 install pillow
  $ pip3 install opencv-contrib-python
  ```

## SETUP

1. Clone this repository inside the `~/ros2_ws/src` folder (ROS workspace).
   ```bash
   $ git clone https://github.com/mk04366/roboracer-autodrive.git
   ```
2. Build the packages.
   ```bash
   $ cd ~/ros2_ws
   $ colcon build
   ```

## USAGE

There are multiple algorithms implemented for each domain in their respective directories. In order to run the whole system, there is a dedicated package `system_launch` to instantiate all the required nodes together. Currently, they can be flexibly configured together for testing out different combinations.

Full system launch can be debugged and visualized via both `foxglove` and `rviz` and the user can run the launch file of their preference. Although, please make sure to keep the required nodes in the file before build and launch and run the autodrive simulator from `autodrive_simulator` folder to connect with the ROS node via the bridge.

- **Bringup:**
  - **Foxglove Bringup:**
    ```bash
    $ ros2 launch system_launch full_system_launch_foxglove.launch.py
    ```
    **[OR]**
  - **RViz Mode Bringup:**
    ```bash
    $ ros2 launch system_launch full_system_launch_rviz.launch.py
    ```

- **Teleoperation:**
  ```bash
  $ ros2 run autodrive_f1tenth teleop_keyboard
  ```
