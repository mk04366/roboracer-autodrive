#!/usr/bin/env python3
"""
Simple MPCC-style controller for F1TENTH AutoDrive (ROS2, Python)

- Uses a lightweight random-shooting optimizer (no external deps) so it's easy to run.
- Subscribe to:
  - /ips (geometry_msgs/Point) for position (x,y,z)
  - /imu (sensor_msgs/Imu) optionally for heading/velocity
  - /scan (sensor_msgs/LaserScan) optional for future obstacle avoidance
- Publish to:
  - /steering_command (std_msgs/Float32)  -- steering in range [-1,1] (normalized)
  - /throttle_command (std_msgs/Float32)  -- throttle in range [-1,1] (normalized)

Parameters (ROS2 parameters):
- waypoints_path: path to csv containing waypoints (x,y) for global trajectory
- control_rate: loop frequency Hz
- horizon: number of steps in prediction horizon
- dt: timestep for prediction
- wheelbase: vehicle wheelbase (m)
- delta_max_deg: max steering radians = deg2rad(param)
- a_max: max accel for mapping throttle
- v_max: expected max speed
- num_samples: how many random-control sequences to sample each iteration

This is intended as a simple, easy-to-understand baseline. Once working,
we can replace the optimizer with GRAMPC/CasADi for speed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
import os

import numpy as np
import csv
import math
import time


# -------------------------
# Utility: path helpers
# -------------------------

def load_waypoints_from_csv(path):
    pts = []
    try:
        with open(path, 'r') as fh:
            rdr = csv.reader(fh)
            for row in rdr:
                if len(row) < 2:
                    continue
                x = float(row[0]); y = float(row[1])
                pts.append((x, y))
    except Exception as e:
        print("Failed to load waypoints:", e)
    return np.array(pts)


def nearest_point_on_path(traj, x, y):
    # returns index, t, proj (x,y), s along path
    best_d2 = 1e12
    best = (0, 0.0, traj[0].copy(), 0.0)
    if traj.shape[0] < 2:
        return best
    seg_lens = np.linalg.norm(np.diff(traj, axis=0), axis=1)
    cumlen = np.concatenate([[0.0], np.cumsum(seg_lens)])
    for i in range(len(traj)-1):
        a = traj[i]
        b = traj[i+1]
        ab = b - a
        denom = np.dot(ab, ab)
        if denom == 0:
            t = 0.0
        else:
            t = np.dot(np.array([x, y]) - a, ab) / denom
            t = float(np.clip(t, 0.0, 1.0))
        proj = a + t * ab
        d2 = (proj[0]-x)**2 + (proj[1]-y)**2
        if d2 < best_d2:
            s = cumlen[i] + t * np.linalg.norm(ab)
            best = (i, t, proj, s)
            best_d2 = d2
    return best


def path_tangent(traj, idx):
    if idx >= len(traj)-1:
        idx = len(traj)-2
    a = traj[idx]
    b = traj[idx+1]
    dx, dy = b - a
    return math.atan2(dy, dx)


def curvature_at(traj, idx):
    # three-point discrete curvature
    n = len(traj)
    if n < 3:
        return 0.0
    i = max(1, min(idx, n-2))
    p_prev = traj[i-1]; p = traj[i]; p_next = traj[i+1]
    a = p_prev; b = p; c = p_next
    area = abs(np.cross(b-a, c-a)) / 2.0
    ab = np.linalg.norm(b-a)
    bc = np.linalg.norm(c-b)
    ca = np.linalg.norm(c-a)
    if area < 1e-9 or min(ab, bc, ca) < 1e-6:
        return 0.0
    R = (ab * bc * ca) / (4.0 * area)
    if R == 0:
        return 0.0
    return 1.0 / R


# -------------------------
# Vehicle model (kinematic)
# -------------------------

def kinematic_step(state, control, dt, L, delta_max, a_max):
    # state: [x,y,theta,v]
    x, y, th, v = state
    steer_norm, thr_norm = control  # in [-1,1]
    delta = steer_norm * delta_max
    a = thr_norm * a_max
    x_next = x + v * math.cos(th) * dt
    y_next = y + v * math.sin(th) * dt
    th_next = th + (v / L) * math.tan(delta) * dt
    v_next = v + a * dt
    if v_next < 0:
        v_next = 0.0
    return np.array([x_next, y_next, th_next, v_next])


def rollout(state0, U_seq, dt, L, delta_max, a_max):
    s = state0.copy()
    states = [s.copy()]
    for u in U_seq:
        s = kinematic_step(s, u, dt, L, delta_max, a_max)
        states.append(s.copy())
    return np.array(states)


# -------------------------
# Cost for MPCC (simple)
# -------------------------

def compute_cost(states, U_seq, traj, v_max, weights):
    # states: (N+1,4), U_seq: (N,2)
    cost = 0.0
    N = U_seq.shape[0]
    for k in range(N):
        xk, yk, thk, vk = states[k]
        idx, t, proj, s = nearest_point_on_path(traj, xk, yk)
        theta_r = path_tangent(traj, idx)
        tangent = np.array([math.cos(theta_r), math.sin(theta_r)])
        normal = np.array([-math.sin(theta_r), math.cos(theta_r)])
        dx = np.array([xk, yk]) - proj
        e_l = float(np.dot(dx, tangent))
        e_c = float(np.dot(dx, normal))
        kappa = curvature_at(traj, idx)
        v_ref = v_max / (1.0 + weights['kappa_gain'] * kappa)
        cost += weights['wc'] * (e_c**2) + weights['wl'] * (e_l**2)
        cost += weights['wv'] * ((vk - v_ref)**2)
        u = U_seq[k]
        cost += weights['wu'] * (u[0]**2 + u[1]**2)
        if k > 0:
            du = U_seq[k] - U_seq[k-1]
            cost += weights['wdu'] * np.sum(du**2)
    # terminal position
    xf, yf, thf, vf = states[-1]
    idx, t, proj, s = nearest_point_on_path(traj, xf, yf)
    theta_r = path_tangent(traj, idx)
    tangent = np.array([math.cos(theta_r), math.sin(theta_r)])
    normal = np.array([-math.sin(theta_r), math.cos(theta_r)])
    dx = np.array([xf, yf]) - proj
    e_l = float(np.dot(dx, tangent))
    e_c = float(np.dot(dx, normal))
    cost += weights['terminal'] * (e_c**2 + e_l**2)
    return float(cost)


# -------------------------
# Simple random-shooting optimizer
# -------------------------

def random_shooting_opt(state0, traj, N, dt, L, delta_max, a_max,
                         v_max, num_samples, prev_U, weights):
    # prev_U: warm start array (N,2) or None
    best_cost = float('inf')
    best_U = None
    # sample around warm start if available
    for i in range(num_samples):
        if prev_U is None:
            # sample each control uniformly in [-1,1]
            U = np.random.uniform(-1.0, 1.0, size=(N,2))
        else:
            # gaussian perturbation around prev_U
            U = prev_U + 0.1 * np.random.randn(N,2)
            U = np.clip(U, -1.0, 1.0)
        states = rollout(state0, U, dt, L, delta_max, a_max)
        c = compute_cost(states, U, traj, v_max, weights)
        if c < best_cost:
            best_cost = c
            best_U = U
    return best_U, best_cost


# -------------------------
# ROS2 Node
# -------------------------
class MPCCNode(Node):
    def __init__(self):
        super().__init__('mpcc_controller')
        # Parameters (with defaults)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('horizon', 8)
        self.declare_parameter('dt', 0.12)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('delta_max_deg', 25.0)
        self.declare_parameter('a_max', 3.0)
        self.declare_parameter('v_max', 4.0)
        self.declare_parameter('num_samples', 400)

        trajectory_path = os.path.join(
            get_package_share_directory('control_py'),
            'config',
            'traj_race_cl.csv'
        )
        self.get_logger().info(f'Loading trajectory from: {trajectory_path}')
        self.traj = load_waypoints_from_csv(trajectory_path) if trajectory_path else np.zeros((0,2))
        
        self.control_rate = float(self.get_parameter('control_rate').get_parameter_value().double_value)
        self.N = int(self.get_parameter('horizon').get_parameter_value().integer_value)
        self.dt = float(self.get_parameter('dt').get_parameter_value().double_value)
        self.L = float(self.get_parameter('wheelbase').get_parameter_value().double_value)
        self.delta_max = math.radians(float(self.get_parameter('delta_max_deg').get_parameter_value().double_value))
        self.a_max = float(self.get_parameter('a_max').get_parameter_value().double_value)
        self.v_max = float(self.get_parameter('v_max').get_parameter_value().double_value)
        self.num_samples = int(self.get_parameter('num_samples').get_parameter_value().integer_value)

        # weights
        self.weights = {
            'wc': 200.0,
            'wl': 1.0,
            'wv': 1.0,
            'wu': 0.5,
            'wdu': 5.0,
            'kappa_gain': 5.0,
            'terminal': 100.0
        }

        # state (x,y,theta,v)
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.have_pose = False
        self.have_imu = False

        # warm start
        self.prev_U = None

        # subscribers
        self.create_subscription(Point, '/autodrive/f1tenth_1/ips', self.ips_cb, 10)
        self.create_subscription(Imu, '/autodrive/f1tenth_1/imu', self.imu_cb, 10)
        # optional scan
        self.create_subscription(LaserScan, '/autodrive/f1tenth_1/scan', self.scan_cb, 10)

        # publishers
        self.pub_steer = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.pub_throttle = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)

        # timer for control loop
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.get_logger().info('MPCC controller node started')

    def ips_cb(self, msg: Point):
        # localization provides x,y,z. we keep history for heading estimation if IMU not present
        x = float(msg.x); y = float(msg.y)
        # simple heading/speed estimate from last pose if no IMU
        if not hasattr(self, 'last_pose'):
            self.last_pose = (x, y, time.time())
        else:
            x_prev, y_prev, t_prev = self.last_pose
            t_now = time.time()
            dt = max(1e-6, t_now - t_prev)
            vx = (x - x_prev) / dt
            vy = (y - y_prev) / dt
            v = math.hypot(vx, vy)
            theta = math.atan2(y - y_prev, x - x_prev) if v > 0.01 else self.state[2]
            self.state[0] = x
            self.state[1] = y
            self.state[2] = theta
            self.state[3] = v
            self.last_pose = (x, y, t_now)
            self.have_pose = True

    def imu_cb(self, msg: Imu):
        # if IMU provides orientation, we can extract yaw
        # msg.orientation is quaternion
        q = msg.orientation
        # quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # linear accel could be used for better speed est
        self.state[2] = yaw
        self.have_imu = True

    def scan_cb(self, msg: LaserScan):
        # placeholder in case you want to add obstacle avoidance later
        self.last_scan = msg

    def control_loop(self):
        if not self.have_pose:
            self.get_logger().warning('No pose yet, waiting for /ips')
            return
        if self.traj.shape[0] < 2:
            self.get_logger().warning('No trajectory loaded, set waypoints_path param to CSV of x,y')
            return

        # run optimizer
        state0 = self.state.copy()
        # warm start: shift prev_U
        if self.prev_U is not None:
            U0 = np.vstack((self.prev_U[1:], np.zeros((1,2))))
        else:
            U0 = None

        best_U, best_cost = random_shooting_opt(state0, self.traj, self.N, self.dt,
                                                self.L, self.delta_max, self.a_max,
                                                self.v_max, self.num_samples, U0, self.weights)
        if best_U is None:
            self.get_logger().error('Optimization failed to produce a control')
            return
        self.prev_U = best_U
        first_u = best_U[0]
        steer_norm = float(np.clip(first_u[0], -1.0, 1.0))
        throttle_norm = float(np.clip(first_u[1], -1.0, 1.0))

        # publish as normalized commands in [-1,1]
        msg_s = Float32()
        msg_t = Float32()
        msg_s.data = steer_norm
        msg_t.data = throttle_norm
        self.pub_steer.publish(msg_s)
        self.pub_throttle.publish(msg_t)

        # debug logging
        self.get_logger().debug(f'Published steer={steer_norm:.3f}, throttle={throttle_norm:.3f}, cost={best_cost:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MPCCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
