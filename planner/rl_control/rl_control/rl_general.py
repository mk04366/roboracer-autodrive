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
from sensor_msgs.msg import LaserScan  # Lidar
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Imu

# Generalized RL Environment: Uses only lidar, speed, and IMU
# No position (IPS) data - makes the model track-agnostic

class AutodriveEnv(gym.Env):
    """
    Generalized RL Environment for autonomous racing.
    
    Inputs: Lidar (72 points), Speed (1), IMU (10)
    Outputs: Steering [-1, 1], Throttle [-1, 1]
    
    Goal: Train a model that generalizes to any track and avoids obstacles.
    """
        
    def __init__(self, node_name='rl_env_node'):
        # 1. Initialize the ROS 2 Node (IMPORTANT: Env class acts as a Node)
        self.node = Node(node_name)
        
        # 2. Define Action and Observation Spaces (Core of the RL problem)
        
        # Action Space (Continuous control: Steering command and Throttle command)
        # Steering: [-1.0, 1.0], Throttle: [-1.0, 1.0] (allows braking)
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), 
                                       high=np.array([1.0, 1.0]), 
                                       dtype=np.float32)
        
        # Observation Space: Lidar + Speed + IMU (NO IPS - track agnostic!)
        # Increased Lidar from 36 to 72 for better obstacle detection
        self.LIDAR_POINTS = 72  # Downsampling 1080 -> 72 (factor of 15)
        self.IMU_READINGS = 10  # 4 Orientation (Quaternion) + 3 AngVel + 3 LinAcc
        TOTAL_OBSERVATION_SIZE = self.LIDAR_POINTS + 1 + self.IMU_READINGS  # 72 + 1 + 10 = 83

        self.observation_space = spaces.Box(low=-1.0,
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

        # Subscriber for sensor data (Lidar)
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

        # Internal state storage
        self.latest_observation = np.zeros(TOTAL_OBSERVATION_SIZE, dtype=np.float32)
        self.current_speed = 0.0
        self.is_done = False
        self.last_steering = 0.0  # Track previous steering for smoothness penalty
        self.last_throttle = 0.0  # Track previous throttle for smoothness
        self.step_count = 0  # Track steps in episode for survival bonus
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
        
        # Downsample Lidar (Take every 15th point for better resolution)
        # 1080 / 15 = 72 points
        step_size = 15
        downsampled_scan = clamped_scan_data[::step_size][:self.LIDAR_POINTS]  # Ensure we get exactly 72 points
        
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
        if np.min(self.latest_observation[:self.LIDAR_POINTS]) < -0.96: 
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
        # Lidar: 0-71 (72 points)
        # Speed: 72
        # IMU: 73-82 (10 values)
        
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

    # NOTE: IPS callback removed - model is now track-agnostic
    # The model learns to drive based only on local sensor data (lidar, speed, IMU)
    # This allows generalization to any track layout
        
    # --- GYMNASIUM REQUIRED METHODS ---

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # 1. Reset Internal State (important for a new episode)
        self.is_done = False
        self.current_speed = 0.0
        self.last_steering = 0.0
        self.last_throttle = 0.0
        self.step_count = 0
        
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
            throttle = np.clip(throttle * 1.2, 0.0, 1.0)  # Boost slightly
        
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

        # Increment step count for survival tracking
        self.step_count += 1

        # 3. Calculate Reward
        # Reward is based on the new state
        reward = self._calculate_reward(action) 
        
        # Update tracking for smoothness penalty
        self.last_steering = steer
        self.last_throttle = throttle 
        
        # 4. Prepare return values
        observation = self.latest_observation.copy()
        terminated = self.is_done 
        truncated = False  # Use truncated for time limits/step limits
        info = {}

        if not np.all(np.isfinite(self.latest_observation)):
            self.node.get_logger().error(f"Invalid observation: {self.latest_observation}")

        self.node.get_logger().info(f"Action: [{action[0]:.2f}, {action[1]:.2f}], Reward: {reward:.4f}, Step: {self.step_count}")

        return observation, reward, terminated, truncated, info

    def _calculate_reward(self, action):
        """
        Speed-focused reward function for track-agnostic driving.
        
        Key objectives:
        1. GO FAST - speed is the primary reward
        2. Avoid walls/obstacles (only penalize when actually dangerous)
        3. Drive smoothly (minimize jerky movements)
        
        Distance calibration (based on actual measurements):
        - At track center: min_dist ≈ 0.55m → normalized ≈ -0.90
        - Collision threshold: ~0.25m → normalized ≈ -0.96
        """
        lidar_data = self.latest_observation[:self.LIDAR_POINTS]
        min_dist = np.min(lidar_data)
        
        # --- 1. SPEED REWARD (PRIMARY) ---
        # Strong speed reward - this is the main objective
        speed_reward = self.current_speed * 3.0  # Max ~15 at full speed
        
        # Penalty for being too slow (encourages forward progress)
        if self.current_speed < 0.5:
            speed_reward -= 2.0
        
        # --- 2. WALL AVOIDANCE (Only penalize when actually close) ---
        # Calibrated based on actual track measurements:
        # - Track center: ~0.55m → normalized -0.90
        # - Close to wall: ~0.35m → normalized -0.93
        # - Very close: ~0.25m → normalized -0.95
        # - Collision: ~0.20m → normalized -0.96
        
        VERY_CLOSE = -0.95    # ~0.25m - getting dangerous
        CLOSE = -0.93         # ~0.35m - too close for comfort
        NORMAL = -0.90        # ~0.55m - track center (no penalty here!)
        
        if min_dist < VERY_CLOSE:
            # Very close to wall - strong penalty
            wall_penalty = -8.0 * (VERY_CLOSE - min_dist) / (VERY_CLOSE - (-1.0))
        elif min_dist < CLOSE:
            # Getting close - moderate penalty
            wall_penalty = -3.0 * (CLOSE - min_dist) / (CLOSE - VERY_CLOSE)
        elif min_dist < NORMAL:
            # Slightly closer than center - small penalty
            wall_penalty = -0.5 * (NORMAL - min_dist) / (NORMAL - CLOSE)
        else:
            # At or beyond track center - no penalty
            wall_penalty = 0.0
        
        # --- 3. SMOOTHNESS PENALTY ---
        # Penalize jerky steering (but less aggressively for racing)
        delta_steering = abs(action[0] - self.last_steering)
        steering_penalty = delta_steering * 0.3
        
        # Penalize rapid throttle changes
        delta_throttle = abs(action[1] - self.last_throttle)
        throttle_penalty = delta_throttle * 0.2
        
        smoothness_penalty = steering_penalty + throttle_penalty
        
        # --- 4. REVERSE PENALTY ---
        # Discourage sustained reverse driving while still allowing braking
        # Braking (brief negative throttle) is OK, but sustained reverse is penalized
        if action[1] < 0:
            # Penalty scales with how much negative throttle is applied
            reverse_penalty = abs(action[1]) * 3.0  # Max -3.0 at full reverse
        else:
            reverse_penalty = 0.0
        
        # --- 5. COLLISION PENALTY ---
        COLLISION_THRESHOLD = -0.9557  # Matches is_done check
        if min_dist < COLLISION_THRESHOLD:
            collision_penalty = -25.0  # Harsh penalty for collision
        else:
            collision_penalty = 0.0
            
        # --- COMBINE ALL REWARDS ---
        total_reward = (
            speed_reward +           # Primary: +0 to +15
            wall_penalty +           # -8 to 0
            collision_penalty -      # -25 or 0
            smoothness_penalty -     # -0 to -0.5
            reverse_penalty          # -0 to -3.0
        )
        
        return total_reward

    def close(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Create the environment
    env = AutodriveEnv()
    check_env(env)
    
    # Define absolute paths
    package_dir = "/home/bl/ros2_ws/src/roboracer-autodrive/planner/rl_control"
    tensorboard_log_dir = os.path.join(package_dir, "tensorboard_logs")
    checkpoint_dir = os.path.join(package_dir, "checkpoints")
    models_dir = os.path.join(package_dir, "models")
    
    # Create necessary directories
    os.makedirs(tensorboard_log_dir, exist_ok=True)
    os.makedirs(checkpoint_dir, exist_ok=True)
    os.makedirs(models_dir, exist_ok=True)
    
    # Model configuration
    run_name = f"PPO_{time.strftime('%Y%m%d_%H%M%S')}"
    final_model_path = os.path.join(models_dir, "rl_general_model.zip")
    
    # Create new PPO model for training
    env.node.get_logger().info("Creating new PPO model for training")
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        device="cpu",
        tensorboard_log=tensorboard_log_dir
    )
    
    # Setup checkpoint callback to save model every 50000 timesteps
    checkpoint_callback = CheckpointCallback(
        save_freq=50000,
        save_path=checkpoint_dir,
        name_prefix=run_name
    )
    
    # Train the model
    try:
        env.node.get_logger().info("Starting training...")
        model.learn(
            total_timesteps=500000,
            callback=checkpoint_callback,
            tb_log_name=run_name
        )
        
        # Save the final trained model
        model.save(final_model_path)
        env.node.get_logger().info(f"Training completed successfully. Model saved to: {final_model_path}")
        
    except KeyboardInterrupt:
        env.node.get_logger().warning("Training interrupted by user")
        interrupted_model_path = os.path.join(checkpoint_dir, f"{run_name}_interrupted.zip")
        model.save(interrupted_model_path)
        env.node.get_logger().info(f"Interrupted model saved to: {interrupted_model_path}")
        
    except Exception as e:
        env.node.get_logger().error(f"Training failed with error: {str(e)}")
        raise
        
    finally:
        env.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()