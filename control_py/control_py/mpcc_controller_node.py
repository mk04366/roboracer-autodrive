#!/usr/bin/env python3
# mpcc_controller.py
# Vectorized random-shooting MPCC-style controller (ROS2, Python)
# Put this file in: ros2_ws/src/control_py/control_py/mpcc_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

import numpy as np
import math
import time
import os
import csv

# ---------- helpers (same as before) ----------
def load_waypoints_from_csv(path):
    pts = []
    try:
        with open(path, 'r') as fh:
            rdr = csv.reader(fh, delimiter=';')
            for row in rdr:
                # skip empty rows or comment lines
                if not row or row[0].strip().startswith('#'):
                    continue
                # skip header line if detected (e.g. contains 'x_m')
                if 'x_m' in row[1]:
                    continue
                # strip spaces and parse floats
                try:
                    x = float(row[1].strip())
                    y = float(row[2].strip())
                    pts.append((x, y))
                except (ValueError, IndexError):
                    # skip rows that can't be parsed
                    continue
    except Exception as e:
        print("Failed to load waypoints:", e)
    
    return np.array(pts)

def nearest_point_on_path(traj, x, y):
    best_d2 = 1e12
    best = (0, 0.0, traj[0].copy(), 0.0)
    if traj.shape[0] < 2:
        return best
    seg_lens = np.linalg.norm(np.diff(traj, axis=0), axis=1)
    cumlen = np.concatenate([[0.0], np.cumsum(seg_lens)])
    for i in range(len(traj)-1):
        a = traj[i]; b = traj[i+1]; ab = b - a
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
    a = traj[idx]; b = traj[idx+1]
    dx, dy = b - a
    return math.atan2(dy, dx)

def curvature_at(traj, idx):
    n = len(traj)
    if n < 3:
        return 0.0
    i = max(1, min(idx, n-2))
    p_prev = traj[i-1]; p = traj[i]; p_next = traj[i+1]
    a = p_prev; b = p; c = p_next
    area = abs(np.cross(b-a, c-a)) / 2.0
    ab = np.linalg.norm(b-a); bc = np.linalg.norm(c-b); ca = np.linalg.norm(c-a)
    if area < 1e-9 or min(ab, bc, ca) < 1e-6:
        return 0.0
    R = (ab * bc * ca) / (4.0 * area)
    if R == 0:
        return 0.0
    return 1.0 / R

# ---------- vehicle model ----------
def kinematic_step_vec(states, controls, dt, L, delta_max, a_max):
    # vectorized: states: (M,4), controls: (M,2) -> next states (M,4)
    x = states[:,0]; y = states[:,1]; th = states[:,2]; v = states[:,3]
    steer_norm = controls[:,0]; thr_norm = controls[:,1]
    delta = steer_norm * delta_max
    a = thr_norm * a_max
    x_next = x + v * np.cos(th) * dt
    y_next = y + v * np.sin(th) * dt
    th_next = th + (v / L) * np.tan(delta) * dt
    v_next = v + a * dt
    v_next = np.maximum(v_next, 0.0)
    return np.stack([x_next, y_next, th_next, v_next], axis=1)

def rollout_vectorized(state0, U_seq): 
    # U_seq: (num_samples, N, 2)
    # state0: (4,) initial state -> we expand to (num_samples,4)
    num_samples, N, _ = U_seq.shape
    states = np.zeros((num_samples, N+1, 4), dtype=float)
    states[:,0,:] = state0.reshape(1,4)
    return states,  # we'll compute iteratively below in optimizer

# ---------- vectorized cost + shooting optimizer ----------
def evaluate_samples_vectorized(state0, traj, U_samples, dt, L, delta_max, a_max, v_max, weights):
    # U_samples: (S, N, 2)
    S, N, _ = U_samples.shape
    # expand states
    states = np.zeros((S, N+1, 4), dtype=float)
    states[:,0,:] = np.tile(state0.reshape(1,4), (S,1))
    # propagate (vectorized per step)
    for k in range(N):
        states[:,k+1,:] = kinematic_step_vec(states[:,k,:], U_samples[:,k,:], dt, L, delta_max, a_max)
    # compute costs (vectorized)
    costs = np.zeros((S,), dtype=float)
    for k in range(N):
        pts = states[:,k,:2]  # (S,2)
        # for each sample, compute nearest path index/proj (this part is still looped per sample; optimize by vectorizing approximations)
        # We'll compute cost by iterating samples — but we removed inner-most horizon loop; still better than full python loops.
        pass
    # To keep code simple and robust we do per-sample cost in numpy loops (S loop). S should be <= ~200
    for i in range(S):
        cost = 0.0
        for k in range(N):
            xk, yk, thk, vk = states[i,k]
            idx, t, proj, s = nearest_point_on_path(traj, xk, yk)
            theta_r = path_tangent(traj, idx)
            tangent = np.array([math.cos(theta_r), math.sin(theta_r)])
            normal = np.array([-math.sin(theta_r), math.cos(theta_r)])
            dx = np.array([xk, yk]) - proj
            e_l = float(np.dot(dx, tangent))
            e_c = float(np.dot(dx, normal))
            kappa = curvature_at(traj, idx)
            v_ref = v_max / (1.0 + weights['kappa_gain'] * kappa)
            u = U_samples[i,k]
            cost += weights['wc'] * (e_c**2) + weights['wl'] * (e_l**2)
            cost += weights['wv'] * ((vk - v_ref)**2)
            cost += weights['wu'] * (u[0]**2 + u[1]**2)
            if k > 0:
                du = U_samples[i,k] - U_samples[i,k-1]
                cost += weights['wdu'] * np.sum(du**2)
        # terminal
        xf, yf, thf, vf = states[i,-1]
        idx, t, proj, s = nearest_point_on_path(traj, xf, yf)
        theta_r = path_tangent(traj, idx)
        tangent = np.array([math.cos(theta_r), math.sin(theta_r)])
        normal = np.array([-math.sin(theta_r), math.cos(theta_r)])
        dx = np.array([xf, yf]) - proj
        e_l = float(np.dot(dx, tangent)); e_c = float(np.dot(dx, normal))
        cost += weights['terminal'] * (e_c**2 + e_l**2)
        costs[i] = cost
    return costs, states

def vectorized_random_shooting(state0, traj, N, dt, L, delta_max, a_max, v_max, num_samples, prev_U, weights):
    # Build sample set S x N x 2
    S = int(num_samples)
    if prev_U is None:
        # steer in [-1,1], throttle in [0,1]
        U_samples = np.zeros((S, N, 2))
        U_samples[...,0] = np.random.uniform(-1.0, 1.0, size=(S, N))
        U_samples[...,1] = np.random.uniform( 0.0, 1.0, size=(S, N))
    else:
        base = np.vstack((prev_U[1:], np.zeros((1,2))))
        base[:,1] = np.clip(base[:,1], 0.0, 1.0)  # ensure throttle >=0
        U_samples = base[None,:,:] + 0.12 * np.random.randn(S, N, 2)
        U_samples[...,1] = np.clip(U_samples[...,1], 0.0, 1.0)
        U_samples[...,0] = np.clip(U_samples[...,0], -1.0, 1.0)

    costs, states = evaluate_samples_vectorized(state0, traj, U_samples, dt, L, delta_max, a_max, v_max, weights)
    best_idx = int(np.argmin(costs))
    best_U = U_samples[best_idx]
    best_states = states[best_idx]
    return best_U, best_states, float(costs[best_idx])

# ---------- ROS2 Node ----------
class MPCCNode(Node):
    def __init__(self):
        super().__init__('mpcc_controller')
        # tuned defaults inspired by reference implementation
        self.declare_parameter('horizon', 3)
        self.declare_parameter('dt', 0.2)  
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('delta_max_deg', 25.0)
        self.declare_parameter('a_max', 3.0)
        self.declare_parameter('v_max', 4.0)
        self.declare_parameter('num_samples', 20)  # much smaller than 400

        trajectory_path = os.path.join(
            get_package_share_directory('control_py'),
            'config',
            'traj_race_cl.csv'
        )
        self.get_logger().info(f'Loading trajectory from: {trajectory_path}')
        self.traj = load_waypoints_from_csv(trajectory_path) if os.path.exists(trajectory_path) else np.zeros((0,2))

        self.N = int(self.get_parameter('horizon').get_parameter_value().integer_value)
        self.dt = float(self.get_parameter('dt').get_parameter_value().double_value)
        self.L = float(self.get_parameter('wheelbase').get_parameter_value().double_value)
        self.delta_max = math.radians(float(self.get_parameter('delta_max_deg').get_parameter_value().double_value))
        self.a_max = float(self.get_parameter('a_max').get_parameter_value().double_value)
        self.v_max = float(self.get_parameter('v_max').get_parameter_value().double_value)
        self.num_samples = int(self.get_parameter('num_samples').get_parameter_value().integer_value)

        self.weights = {
            'wc': 80.0,
            'wl': 1.0,
            'wv': 1.0,
            'wu': 1.0,
            'wdu': 10.0,
            'kappa_gain': 5.0,
            'terminal': 50.0
        }

        self.state = np.array([0.0,0.0,0.0,0.0])
        self.have_pose = False
        self.have_imu = False
        self.prev_U = None

        self.create_subscription(Point, '/autodrive/f1tenth_1/ips', self.ips_cb, 10)
        self.create_subscription(Imu, '/autodrive/f1tenth_1/imu', self.imu_cb, 10)
        self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_cb, 10)

        self.pub_steer = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.pub_throttle = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)

        # control loop runs at fixed rate regardless of sensor rates
        self.control_rate = 1.0 / 0.02  # target 50 Hz (timer period 0.02)
        self.timer = self.create_timer(0.02, self.control_loop)

        # logging flags
        self.warned_no_pose = False

        self.get_logger().info('MPCC controller (vectorized) started')

    def ips_cb(self, msg: Point):
        x = float(msg.x); y = float(msg.y)
        if not hasattr(self, 'last_pose'):
            self.last_pose = (x, y, time.time())
            return
        x_prev, y_prev, t_prev = self.last_pose
        t_now = time.time()
        dt = max(1e-6, t_now - t_prev)
        vx = (x - x_prev) / dt; vy = (y - y_prev) / dt
        v = math.hypot(vx, vy)
        theta = math.atan2(y - y_prev, x - x_prev) if v > 0.01 else self.state[2]
        self.state[0] = x; self.state[1] = y; self.state[2] = theta; self.state[3] = v
        self.last_pose = (x, y, t_now)
        self.have_pose = True

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.state[2] = yaw; self.have_imu = True

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def control_loop(self):
        if not self.have_pose:
            if not self.warned_no_pose:
                self.get_logger().warning('No pose yet, waiting for /autodrive/f1tenth_1/ips')
                self.warned_no_pose = True
            return
        if self.traj.shape[0] < 2:
            self.get_logger().warning('No trajectory loaded in package control_py/config/')
            return

        state0 = self.state.copy()
        # warm start shift
        if self.prev_U is not None:
            warm = np.vstack((self.prev_U[1:], np.zeros((1,2))))
        else:
            warm = None

        start = time.time()
        best_U, best_states, best_cost = vectorized_random_shooting(state0, self.traj, self.N, self.dt,
                                                                    self.L, self.delta_max, self.a_max,
                                                                    self.v_max, self.num_samples, warm, self.weights)
        elapsed_ms = (time.time() - start) * 1000.0
        self.get_logger().info(f"[MPCC] Solve time: {elapsed_ms:.1f} ms, samples={self.num_samples}, N={self.N}")

        if best_U is None:
            self.get_logger().error('No solution from optimizer')
            return

        self.prev_U = best_U.copy()
        first = best_U[0]
        steer_norm = float(np.clip(first[0], -1.0, 1.0))
        thr_norm = float(np.clip(first[1], -0.1, 1.0))

        msg_s = Float32(); msg_t = Float32()
        msg_s.data = steer_norm; msg_t.data = thr_norm
        self.pub_steer.publish(msg_s); self.pub_throttle.publish(msg_t)

        # optional debug: log predicted head of best_states
        # head = best_states[1]  # first predicted state after current
        # self.get_logger().debug(f'Predicted next pos: {head[0]:.2f},{head[1]:.2f}, v={head[3]:.2f}')

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
