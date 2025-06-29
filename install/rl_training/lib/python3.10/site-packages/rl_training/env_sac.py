import time
import threading
import numpy as np
import gym
from gym import spaces

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, Bool


class RoboRacerRLNode(Node):
    """ROS2 Node: subscribe lidar, speed, pose; publish steering/throttle and reset."""
    def __init__(self):
        super().__init__("rl_training_node")

        # State
        self.lidar = None
        self.speed = 0.0
        self.pose = None

        # --- Subscriptions ---
        self.create_subscription(
            LaserScan,
            "/autodrive/f1tenth_1/lidar",
            self.lidar_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/autodrive/f1tenth_1/speed",
            self.speed_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_callback,
            10
        )

        # --- Publishers ---
        self.steering_pub = self.create_publisher(
            Float32, "/autodrive/f1tenth_1/steering_command", 10
        )
        self.throttle_pub = self.create_publisher(
            Float32, "/autodrive/f1tenth_1/throttle_command", 10
        )
        self.reset_pub = self.create_publisher(
            Bool, "/autodrive/reset_command", 10
        )

    # ---------------- Callbacks ----------------
    def lidar_callback(self, msg: LaserScan):
        self.lidar = np.array(msg.ranges, dtype=np.float32)

    def speed_callback(self, msg: Float32):
        self.speed = float(msg.data)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.pose = msg.pose.pose

    # ---------------- Control ----------------
    def publish_action(self, steering: float, throttle: float):
        s = Float32()
        s.data = float(steering)
        self.steering_pub.publish(s)

        t = Float32()
        t.data = float(throttle)
        self.throttle_pub.publish(t)

    def publish_reset(self):
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)


class RoboRacerEnv(gym.Env):
    """Gym SAC Env: track following, avoid collision."""
    metadata = {"render.modes": []}

    def __init__(self, lidar_clip=10.0, max_episode_steps=800):
        super().__init__()

        # --- Init ROS2 ---
        rclpy.init()
        self.node = RoboRacerRLNode()

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        self.spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self.spin_thread.start()

        # Wait lidar ready
        self._wait_lidar()

        self.lidar_clip = lidar_clip
        self.max_episode_steps = max_episode_steps
        self.step_count = 0

        # Observation space
        lidar_dim = self.node.lidar.shape[0]
        self.lidar_dim = lidar_dim

        obs_dim = lidar_dim + 1   # lidar + speed

        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(obs_dim,),
            dtype=np.float32
        )

        # Action space: steering ∈ [-1,1], throttle ∈ [0,1]
        self.action_space = spaces.Box(
            low=np.array([-1.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

    def _wait_lidar(self):
        self.node.get_logger().info("Waiting for LiDAR...")
        while rclpy.ok() and self.node.lidar is None:
            time.sleep(0.05)
        self.node.get_logger().info(f"Got lidar, dim={self.node.lidar.shape[0]}")

    # ---------------- Gym API ----------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0

        self.node.publish_reset()
        time.sleep(1.0)

        self._wait_lidar()
        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        self.step_count += 1

        action = np.clip(action, self.action_space.low, self.action_space.high)

        steering = float(action[0]) * 0.5       # → [-0.5, 0.5] rad
        throttle = float(action[1])             # → [0,1]

        self.node.publish_action(steering, throttle)

        time.sleep(0.05)

        obs = self._get_obs()
        reward, done, info = self._compute_reward(obs)

        if self.step_count >= self.max_episode_steps:
            done = True

        return obs, reward, done, False, info

    # ---------------- State & Reward ----------------
    def _get_obs(self):
        lidar = np.copy(self.node.lidar)
        lidar = np.nan_to_num(lidar, nan=self.lidar_clip)
        lidar = np.clip(lidar, 0.0, self.lidar_clip) / self.lidar_clip

        speed = np.array([self.node.speed], dtype=np.float32)

        return np.concatenate([lidar.astype(np.float32), speed])

    def _compute_reward(self, obs):
        lidar = obs[:self.lidar_dim] * self.lidar_clip
        speed = float(obs[-1])

        min_dist = float(np.min(lidar))
        left = float(np.mean(lidar[:180]))
        right = float(np.mean(lidar[-180:]))

        r_speed = speed
        r_safety = 0.5 * min_dist
        r_center = -0.1 * abs(left - right)

        reward = r_speed + r_safety + r_center
        done = False
        info = {"min_dist": min_dist}

        if min_dist < 0.2:
            reward -= 50.0
            info["crash"] = True
            done = True

        return reward, done, info

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join()