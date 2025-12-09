import rclpy
from rclpy.node import Node
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from stable_baselines3 import PPO
import os
import time
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import CheckpointCallback

# Import messages used for control and sensing
from sensor_msgs.msg import LaserScan # Lidar
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu

# Import other relevant messages for state/reward calculation (e.g., Odometry, CarState, etc.)

class AutodriveEnv(gym.Env):
        
    def __init__(self, node_name='rl_env_node'):
        # 1. Initialize the ROS 2 Node (IMPORTANT: Env class acts as a Node)
        self.node = Node(node_name)
        
        # 2. Define Action and Observation Spaces (Core of the RL problem)
        
        # Action Space (Continuous control: Steering command and Throttle command)
        # Assuming steering is [-1.0, 1.0] and throttle is [0.0, 1.0]
        self.action_space = spaces.Box(low=np.array([-1.0, 0.0]), 
                                       high=np.array([1.0, 1.0]), 
                                       dtype=np.float32)
        
        # Observation Space (Lidar data (36 readings) + Current Speed (1 reading) + IMU (10 readings) + IPS (3 readings))
        # Downsampling Lidar from 1080 to 36 (Factor of 30)
        self.LIDAR_POINTS = 36 
        IMU_READINGS = 10  # 4 for Orientation (Quaternion), 3 for AngVel, 3 for LinAcc
        IPS_READINGS = 3  # x, y, z position
        TOTAL_OBSERVATION_SIZE = self.LIDAR_POINTS + 1 + IMU_READINGS + IPS_READINGS

        self.observation_space = spaces.Box(low=-1.0,  # Adjusted for IMU and IPS ranges
                                            high=1.0, 
                                            shape=(TOTAL_OBSERVATION_SIZE,), 
                                            dtype=np.float32)

        # 3. Setup ROS 2 Publishers and Subscribers
        
        # Publisher for steering command
        self.steering_publisher = self.node.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        
        # Publisher for throttle command
        self.throttle_publisher = self.node.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
                
        # Publisher for reset command
        self.reset_publisher = self.node.create_publisher(Bool, '/autodrive/reset_command', 10)

        # Subscriber for sensor data (Lidar and other car info)
        self.node.create_subscription(LaserScan, 
                                      '/autodrive/f1tenth_1/lidar', 
                                      self._lidar_callback, 
                                      10)
        
        # Subscriber for speed data
        self.node.create_subscription(Float32, 
                                      '/autodrive/f1tenth_1/speed', 
                                      self._speed_callback, 
                                      10)
        
        # Subscriber for IMU data
        self.node.create_subscription(Imu, 
                                      '/autodrive/f1tenth_1/imu', 
                                      self._imu_callback, 
                                      10)

        # Subscriber for IPS data
        self.node.create_subscription(Point, 
                                      '/autodrive/f1tenth_1/ips', 
                                      self._ips_callback, 
                                      10)

        # Internal state storage
        self.latest_observation = np.zeros(TOTAL_OBSERVATION_SIZE, dtype=np.float32)
        self.current_speed = 0.0
        self.is_done = False
        self.last_steering = 0.0 # Track previous steering for smoothness penalty
        # flag for synchronization
        self.new_lidar_received = False

    # --- ROS CALLBACKS ---
    def _lidar_callback(self, msg):
        # Process Lidar data and store it in the observation array
        scan_data = np.array(msg.ranges, dtype=np.float32)
        lidar_len = min(len(scan_data), 1080)
        
        MAX_LIDAR_RANGE = 10.0
        MIN_LIDAR_RANGE = 0.06
        
        clamped_scan_data = np.clip(scan_data[:lidar_len], MIN_LIDAR_RANGE, MAX_LIDAR_RANGE)
        
        # Downsample Lidar (Take every 30th point)
        # 1080 / 30 = 36 points
        step_size = 30
        downsampled_scan = clamped_scan_data[::step_size]
        
        # Normalize Lidar data to [-1, 1]
        # Formula: 2 * ((Value - Min) / (Max - Min)) - 1
        normalized_scan_data = 2 * ((downsampled_scan - MIN_LIDAR_RANGE) / (MAX_LIDAR_RANGE - MIN_LIDAR_RANGE)) - 1
        
        self.latest_observation[:self.LIDAR_POINTS] = normalized_scan_data
        
        # Signal that fresh data is ready
        self.new_lidar_received = True
        
        # Check for collision/terminal state here
        # Collision check for objects too close (Use ANY of the downsampled points)
        # SENSITIVITY UPDATE: Increased threshold to -0.95 to detect crashes EARLIER.
        # This prevents the simulator from "rewinding" before we catch the failure.
        if np.min(self.latest_observation[:self.LIDAR_POINTS]) < -0.95: 
           self.is_done = True

    def _speed_callback(self, msg):
        # Update the current speed from the speed topic
        self.current_speed = msg.data
        
        # Normalize speed to [-1, 1]
        normalized_speed = 2 * ((self.current_speed - 0) / (5 - 0)) - 1
        self.latest_observation[self.LIDAR_POINTS] = normalized_speed

    def _imu_callback(self, msg):
        # Update IMU data in the observation array
        # Indices: 
        # Lidar: 0-1079
        # Speed: 1080
        # IMU: 1081-1090 (10 values)
        # IPS: 1091-1093
        
        # Orientation (Quaternion: x, y, z, w)
        base_idx = self.LIDAR_POINTS + 1 # After Lidar and Speed
        self.latest_observation[base_idx:base_idx+4] = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        
        # Angular Velocity (Normalize to [-1, 1])
        base_idx += 4
        self.latest_observation[base_idx] = 2 * ((msg.angular_velocity.x - (-0.1997)) / (0.2317 - (-0.1997))) - 1
        self.latest_observation[base_idx+1] = 2 * ((msg.angular_velocity.y - (-0.1566)) / (0.1933 - (-0.1566))) - 1
        self.latest_observation[base_idx+2] = 2 * ((msg.angular_velocity.z - (-1.7952)) / (1.5179 - (-1.7952))) - 1
        
        # Linear Acceleration (Normalize to [-1, 1])
        base_idx += 3
        self.latest_observation[base_idx] = 2 * ((msg.linear_acceleration.x - (-7.9472)) / (7.6237 - (-7.9472))) - 1
        self.latest_observation[base_idx+1] = 2 * ((msg.linear_acceleration.y - (-5.4421)) / (5.986 - (-5.4421))) - 1
        self.latest_observation[base_idx+2] = 2 * ((msg.linear_acceleration.z - (-0.2101)) / (0.2638 - (-0.2101))) - 1

    def _ips_callback(self, msg):
        # Update IPS data in the observation array
        # Normalize IPS data to [-1, 1]
        base_idx = self.LIDAR_POINTS + 1 + 10 # After Lidar, Speed, IMU
        self.latest_observation[base_idx] = 2 * ((msg.x - (-3.1241)) / (0.9177 - (-3.1241))) - 1
        self.latest_observation[base_idx+1] = 2 * ((msg.y - (-7.7468)) / (6.106 - (-7.7468))) - 1
        self.latest_observation[base_idx+2] = 2 * ((msg.z - 0.0562) / (0.0605 - 0.0562)) - 1
        
    # --- GYMNASIUM REQUIRED METHODS ---

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # 1. Reset Internal State (important for a new episode)
        self.is_done = False
        self.current_speed = 0.0
        self.last_steering = 0.0
        
        # 2. Command Environment Reset (Autodrive specific)
        reset_msg = Bool()
        reset_msg.data = True
        self.reset_publisher.publish(reset_msg)
        self.node.get_logger().info('Environment reset commanded.')
        
        # Wait briefly to ensure reset is registered
        time.sleep(0.1)

        # Release reset command
        reset_msg.data = False
        self.reset_publisher.publish(reset_msg)
        self.node.get_logger().info('Environment reset released.')
        
        self.node.get_logger().info('Waiting for Lidar data...')
        data_received = False
        while not data_received:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if np.any(self.latest_observation[:self.LIDAR_POINTS] != 0): 
                data_received = True
                
        self.node.get_logger().info('Simulator connected. Lidar data received.')
        
        observation = self.latest_observation.copy()
        info = {}
        return observation, info

    def step(self, action):
        # 1. Publish Action to ROS 2
        
        # Scale the action from the RL model's space to the control message's values
        steer = action[0] 
        throttle = action[1]

        # If the agent wants to move, give it enough juice to actually move.
        if throttle > 0.05:
            throttle = np.clip(throttle * 1.2, 0.0, 1.0) # Boost slightly
        
        # Publish steering and throttle command
        self.steering_publisher.publish(Float32(data=float(steer)))
        self.throttle_publisher.publish(Float32(data=float(throttle)))
        
        # Reset the flag. waiting for the NEXT message.
        self.new_lidar_received = False
        # Safety counter to prevent infinite loops if sim crashes
        wait_counter = 0 
        
        # SPIN UNTIL FRESH: Loop until the callback fires
        while not self.new_lidar_received:
            # Check the subscriber queue rapidly (1ms timeout)
            rclpy.spin_once(self.node, timeout_sec=0.001)
            
            # Safety break after ~1 second (1000 * 0.001)
            wait_counter += 1
            if wait_counter > 1000:
                self.node.get_logger().error("Timeout: Lidar data stopped arriving!")
                # Force a break or handle error (e.g., end episode)
                break

        # 3. Calculate Reward
        # Reward is based on the new state
        reward = self._calculate_reward(action) 
        
        self.last_steering = steer # Update correlation state 
        
        # 4. Prepare return values
        observation = self.latest_observation.copy()
        terminated = self.is_done 
        truncated = False # Use truncated for time limits/step limits
        info = {}

        # Log only the KEY components for easier debugging:
        # Denormalize for logging readability
        # Lidar: [-1, 1] -> [0.06, 10.0]
        min_lidar_norm = np.min(observation[:self.LIDAR_POINTS])
        min_lidar_real = ((min_lidar_norm + 1.0) / 2.0) * (10.0 - 0.06) + 0.06
        
        # Speed: [-1, 1] -> [0.0, 5.0]
        speed_norm = observation[self.LIDAR_POINTS]
        speed_real = ((speed_norm + 1.0) / 2.0) * 5.0

        if not np.all(np.isfinite(self.latest_observation)):
            self.node.get_logger().error(f"Invalid observation: {self.latest_observation}")

        self.node.get_logger().info(f"Action: {action}, Reward: {reward:.4f}")
        self.node.get_logger().info(f"Obs: Min Lidar={min_lidar_real:.4f} m, Speed={speed_real:.4f} m/s")

        return observation, reward, terminated, truncated, info

    def _calculate_reward(self, action):
        # 1. Denormalize Speed for meaningful reward
        normalized_speed = self.latest_observation[self.LIDAR_POINTS] 
        denormalized_speed = (normalized_speed + 1.0) * 2.5 # [0.0 to 5.0 m/s]
        
        # Reward component: Encourage speed 
        # Increase weight to synthesize "Go Fast"
        speed_reward = denormalized_speed * 2.0 
        
        # 2. Wall Distance Reward (Continuous)
        # Encourage keeping a healthy distance from walls
        # Use simple average of downsampled Lidar
        avg_dist = np.mean(self.latest_observation[:self.LIDAR_POINTS])
        distance_reward = (avg_dist + 1) * 0.5 # Shift to positive range roughly
        
        # 3. Smoothness Penalty
        # Penalize large changes in steering
        delta_steering = abs(action[0] - self.last_steering)
        smoothness_penalty = delta_steering * 0.5

        # 4. Collision Penalty
        min_dist = np.min(self.latest_observation[:self.LIDAR_POINTS])
        
        # Penalty threshold: Normalized value corresponding to a raw Lidar reading of ~0.375m (0.25m clearance)
        PENALTY_THRESHOLD = -0.936 
        
        if min_dist < PENALTY_THRESHOLD:
            # Strong penalty to force the agent away from the walls
            collision_penalty = -10.0 
        else:
            collision_penalty = 0.0
            
        # 5. Combine Rewards
        total_reward = speed_reward + distance_reward - smoothness_penalty + collision_penalty
        return total_reward

    def close(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Create the environment
    env = AutodriveEnv()
    # It will check custom environment and output additional warnings if needed
    check_env(env)
    
    # Initialize the agent
    # Enable TensorBoard logging
    # Use path relative to the package root (one level up from this script)
    # File: .../rl_control/rl_control/rl_agent_node.py
    # Dir:  .../rl_control/rl_control/
    # Parent: .../rl_control/
    script_dir = os.path.dirname(os.path.realpath(__file__))
    package_root = os.path.dirname(script_dir)
    
    log_dir = os.path.join(package_root, "tensorboard_logs")
    model = PPO("MlpPolicy", env, verbose=1, device="cpu", tensorboard_log=log_dir)
    
    # Create checkpoint callback
    # Save a checkpoint every 20,000 steps
    save_dir = os.path.join(package_root, "models")
    checkpoint_callback = CheckpointCallback(
        save_freq=20000,
        save_path=save_dir,
        name_prefix="ppo_autodrive_model"
    )
    
    try:
        # Train the agent
        model.learn(total_timesteps=200000, callback=checkpoint_callback)
        
        # Save the model
        model.save("ppo_autodrive")
        
        env.node.get_logger().info("Training finished and model saved.")
        
    except KeyboardInterrupt:
        env.node.get_logger().info("Training interrupted.")
    finally:
        env.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
