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
from visualization_msgs.msg import Marker
# Import other relevant messages for state/reward calculation (e.g., Odometry, CarState, etc.)

class AutodriveEnv(gym.Env):
        
    def __init__(self, node_name='rl_env_node'):
        # 1. Initialize the ROS 2 Node (IMPORTANT: Env class acts as a Node)
        self.node = Node(node_name)
        
        # 2. Define Action and Observation Spaces (Core of the RL problem)
        
        # Action Space (Continuous control: Steering command and Throttle command)
        # Assuming steering is [-1.0, 1.0] and throttle is [0.0, 1.0]
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), 
                                       high=np.array([1.0, 1.0]), 
                                       dtype=np.float32)
        self.current_x = 0.0
        self.current_y = 0.0
        self.previous_index_waypoint = 0
        self.nearest_idx_current = 0 
        
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

        # import waypoints
        self.waypoints = self._load_path_from_csv('/home/bl/ros2_ws/src/roboracer-autodrive/global-planning/outputs/map5/ay_safe_2.csv')
        
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
        self.new_ips_received = False

    def _load_path_from_csv(self, filename):
        waypoints = []
        try:
            with open(filename, 'r') as file:
                for line in file:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    # Replicate std::getline behavior by splitting on comma
                    tokens = line.split(',')
                
                    if len(tokens) >= 6:
                        try:
                            x = float(tokens[1])
                            y = float(tokens[2])
                            psi = float(tokens[3])
                            kappa = float(tokens[4]) # column 4 is kappa
                            v_ref = float(tokens[5]) # column 5 is vx_mps
                            waypoints.append([x, y, psi, kappa, v_ref])
                        except ValueError as e:
                            print(f"Error parsing line: {line} - {e}")
        except Exception as e:
            print(f"Error processing file {filename}: {e}")
            # Return empty path on failure, matching implied C++ behavior
        return waypoints

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
        # SENSITIVITY UPDATE: Increased threshold to -0.95 to detect crashes EARLIER.
        # This prevents the simulator from "rewinding" before we catch the failure.
        if np.min(self.latest_observation[:self.LIDAR_POINTS]) < -0.959: 
           self.is_done = True

    def _speed_callback(self, msg):
        # Update the current speed from the speed topic
        self.current_speed = msg.data
        
        # Normalize speed to [-1, 1]
        normalized_speed = 2 * ((self.current_speed - 0) / (5 - 0)) - 1
        self.latest_observation[self.LIDAR_POINTS] = normalized_speed

    def _imu_callback(self, msg):
        # Update IMU data in the observation array
        
        # Orientation (Quaternion: x, y, z, w) - already normalized [-1, 1]
        base_idx = self.LIDAR_POINTS + 1 # After Lidar and Speed
        self.latest_observation[base_idx:base_idx+4] = np.clip(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            -1.0, 1.0
        )
        
        # Angular Velocity (Normalize to [-1, 1]) - widened bounds with 50% margin
        base_idx += 4
        ANG_VEL_MIN = np.array([-0.3, -0.25, -2.7])  # Widened from original
        ANG_VEL_MAX = np.array([0.35, 0.3, 2.3])     # Widened from original
        ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        ang_vel_normalized = 2 * ((ang_vel - ANG_VEL_MIN) / (ANG_VEL_MAX - ANG_VEL_MIN)) - 1
        self.latest_observation[base_idx:base_idx+3] = np.clip(ang_vel_normalized, -1.0, 1.0)
        
        # Linear Acceleration (Normalize to [-1, 1]) - widened bounds with 50% margin
        base_idx += 3
        LIN_ACC_MIN = np.array([-12.0, -8.0, -0.5])  # Widened from original
        LIN_ACC_MAX = np.array([12.0, 9.0, 0.5])     # Widened from original
        lin_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        lin_acc_normalized = 2 * ((lin_acc - LIN_ACC_MIN) / (LIN_ACC_MAX - LIN_ACC_MIN)) - 1
        self.latest_observation[base_idx:base_idx+3] = np.clip(lin_acc_normalized, -1.0, 1.0)

    def _ips_callback(self, msg):
        # Update IPS data in the observation array
        # Normalize IPS data to [-1, 1] with widened bounds
        base_idx = self.LIDAR_POINTS + 1 + 10 # After Lidar, Speed, IMU
        
        # Widened position bounds based on map extents + 50% margin
        IPS_X_MIN, IPS_X_MAX = -5.0, 3.0   # Widened from (-3.12, 0.92)
        IPS_Y_MIN, IPS_Y_MAX = -12.0, 10.0  # Widened from (-7.75, 6.11)
        IPS_Z_MIN, IPS_Z_MAX = 0.0, 0.2     # Widened from (0.056, 0.061)
        
        # Normalize and clip to ensure bounds
        self.latest_observation[base_idx] = np.clip(
            2 * ((msg.x - IPS_X_MIN) / (IPS_X_MAX - IPS_X_MIN)) - 1, -1.0, 1.0
        )
        self.latest_observation[base_idx+1] = np.clip(
            2 * ((msg.y - IPS_Y_MIN) / (IPS_Y_MAX - IPS_Y_MIN)) - 1, -1.0, 1.0
        )
        self.latest_observation[base_idx+2] = np.clip(
            2 * ((msg.z - IPS_Z_MIN) / (IPS_Z_MAX - IPS_Z_MIN)) - 1, -1.0, 1.0
        )
        self.current_x = msg.x
        self.current_y = msg.y
        self.new_ips_received = True
        
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
        
        self.node.get_logger().info('Waiting for Lidar AND IPS data...')
        lidar_received = False
        # Clear flags before waiting
        self.new_lidar_received = False 
        self.new_ips_received = False # IMPORTANT: Ensure we wait for fresh position data after reset
        
        while not (lidar_received and self.new_ips_received):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
            if np.any(self.latest_observation[:self.LIDAR_POINTS] != 0): 
                lidar_received = True
                
        self.node.get_logger().info('Simulator connected. Lidar and IPS data received.')
        
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

        if not np.all(np.isfinite(self.latest_observation)):
            self.node.get_logger().error(f"Invalid observation: {self.latest_observation}")

        self.node.get_logger().info(f"Action: {action}, Reward: {reward:.4f}")

        return observation, reward, terminated, truncated, info

    def _calculate_traj_tracking_reward(self):
        """
        Improved trajectory tracking reward with balanced components:
        - Progress reward: Encourages forward motion along trajectory
        - Lap completion bonus: Big reward for completing a lap
        - Distance penalty: Penalizes deviation from trajectory (scaled)
        - Direction penalty: Penalizes going backwards
        """
        # Find nearest waypoint
        min_distance = float('inf')
        nearest_point_A = [0, 0]
        nearest_point_B = [0, 0]
        
        for x in range(len(self.waypoints)):
            next_idx = (x + 10) % len(self.waypoints)
            distance = np.linalg.norm(
                np.array([self.current_x, self.current_y]) - 
                np.array([self.waypoints[x][0], self.waypoints[x][1]])
            )
            if distance < min_distance:
                min_distance = distance
                nearest_point_A = [self.waypoints[x][0], self.waypoints[x][1]]
                nearest_point_B = [self.waypoints[next_idx][0], self.waypoints[next_idx][1]]
                self.nearest_idx_current = x
        
        current_point = [self.current_x, self.current_y]
        num_waypoints = len(self.waypoints)
        
        # === PROGRESS REWARD ===
        # Calculate how many waypoints we've advanced (handles wraparound)
        if self.nearest_idx_current >= self.previous_index_waypoint:
            waypoints_advanced = self.nearest_idx_current - self.previous_index_waypoint
        else:
            # Wrapped around the track
            waypoints_advanced = (num_waypoints - self.previous_index_waypoint) + self.nearest_idx_current
        
        # Reward for forward progress (0.5 per waypoint advanced, capped)
        progress_reward = min(waypoints_advanced * 0.5, 5.0)
        
        # === LAP COMPLETION BONUS ===
        lap_bonus = 0.0
        # Detect lap completion: crossed from high index back to low index
        if self.previous_index_waypoint > num_waypoints * 0.9 and self.nearest_idx_current < num_waypoints * 0.1:
            lap_bonus = 100.0  # Big bonus for completing a lap!
            self.node.get_logger().info("🏁 LAP COMPLETED! +100 bonus")
        
        # === DIRECTION PENALTY ===
        direction_penalty = 0.0
        # Going backwards: previous > current, but not due to lap wraparound
        if (self.previous_index_waypoint > self.nearest_idx_current and 
            self.previous_index_waypoint - self.nearest_idx_current < num_waypoints * 0.1):
            direction_penalty = -50.0  # Reduced from -1000 for balance
        
        self.previous_index_waypoint = self.nearest_idx_current
        
        # === DISTANCE PENALTY (perpendicular distance to trajectory line) ===
        x1, y1 = nearest_point_A
        x2, y2 = nearest_point_B
        x3, y3 = current_point

        dx = x2 - x1
        dy = y2 - y1
        
        denominator = np.sqrt(dx**2 + dy**2)
        if denominator > 0:
            numerator = abs(dx * (y1 - y3) + dy * (x3 - x1))
            perpendicular_distance = numerator / denominator
        else:
            perpendicular_distance = min_distance
        
        # Exponential penalty: more forgiving for small deviations, harsh for large ones
        # Scales from 0 (on track) to -5 (far off track)
        MAX_DISTANCE_PENALTY = 5.0
        DISTANCE_SCALE = 2.0  # How quickly penalty increases with distance
        distance_penalty = -MAX_DISTANCE_PENALTY * (1 - np.exp(-DISTANCE_SCALE * perpendicular_distance))
        
        total_reward = progress_reward + lap_bonus + direction_penalty + distance_penalty
        return total_reward


    def _calculate_reward(self, action):
        """
        Tuned reward function with balanced components for trajectory tracking.
        """
    # === DYNAMIC SPEED REWARD ===
        # LOOKAHEAD LOGIC: Check both Reference Velocity AND Curvature
        lookahead_distance = 20
        min_future_speed = float('inf')
        max_future_kappa = 0.0
        
        for i in range(lookahead_distance):
            idx = (self.nearest_idx_current + i) % len(self.waypoints)
            # waypoints structure: [x, y, psi, kappa, v_ref]
            k = abs(self.waypoints[idx][3])
            v_ref = self.waypoints[idx][4]
            
            if v_ref < min_future_speed:
                min_future_speed = v_ref
            if k > max_future_kappa:
                max_future_kappa = k
        
        # 1. Target Speed Guidance (Positive Reward)
        dynamic_target_speed = max(min_future_speed, 1.0)
        
        base_speed_reward = self.current_speed * 1.5
        speed_diff = abs(self.current_speed - dynamic_target_speed)
        target_speed_bonus = 3.0 * np.exp(-0.5 * (speed_diff ** 2))
        speed_reward = base_speed_reward + target_speed_bonus
        
        # 2. Curvature Overspeed Penalty (Negative Reward)
        # If a sharp curve is ahead (kappa > 0.2) and we are going fast, PENALIZE.
        overspeed_penalty = 0.0
        CURVE_THRESHOLD = 0.25
        
        if max_future_kappa > CURVE_THRESHOLD:
            # We are approaching a curve. Speed should be roughly limited by physics/v_ref.
            # Use dynamic_target_speed (derived from v_ref) as the strict limit.
            speed_limit = dynamic_target_speed + 0.5 # Allow 0.5 m/s buffer
            
            if self.current_speed > speed_limit:
                # Strong linear penalty for every m/s or km/h over the limit
                overspeed_penalty = -5.0 * (self.current_speed - speed_limit)
                
                # Cap the penalty to avoid insane values if it goes wild, but keep it high
                overspeed_penalty = max(overspeed_penalty, -20.0) 
                
                if overspeed_penalty < -1.0:
                    self.node.get_logger().debug(
                        f"⚠️ OVERSPEED PENALTY! Curve (k={max_future_kappa:.2f}) ahead. "
                        f"Speed: {self.current_speed:.2f} > Limit: {speed_limit:.2f}. Pen: {overspeed_penalty:.2f}"
                    )
        
        # === SMOOTHNESS PENALTY ===
        # Penalize jerky steering more aggressively with exponential scaling
        delta_steering = abs(action[0] - self.last_steering)
        
        # Small changes (< 0.1): minimal penalty
        # Medium changes (0.1-0.5): moderate penalty  
        # Large changes (> 0.5): severe penalty
        if delta_steering < 0.1:
            smoothness_penalty = delta_steering * 1.0  # Light penalty for micro-adjustments
        elif delta_steering < 0.5:
            smoothness_penalty = 0.1 + (delta_steering - 0.1) * 3.0  # Medium penalty
        else:
            smoothness_penalty = 1.3 + (delta_steering - 0.5) * 8.0  # Harsh penalty for jerky moves

        # reward for following track
        traj_tracking_reward = self._calculate_traj_tracking_reward()
        
        # === CRASH PENALTY ===
        crash_penalty = 0.0
        if self.is_done:
             crash_penalty = -100.0
             self.node.get_logger().info("💥 CRASH DETECTED! Applying -100 penalty.")
            
        # Combine Rewards
        total_reward = speed_reward + overspeed_penalty - smoothness_penalty + traj_tracking_reward + crash_penalty
        
        # Debug logging
        self.node.get_logger().debug(
            f"Reward: speed={speed_reward:.2f}, overspeed={overspeed_penalty:.2f}, smooth={smoothness_penalty:.2f}, "
            f"traj={traj_tracking_reward:.2f}, crash={crash_penalty:.2f}, total={total_reward:.2f}"
        )
        
        return total_reward

    def close(self):
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Create the environment
    env = AutodriveEnv()
    # It will check custom environment and output additional warnings if needed
    check_env(env)
    
    # Save directory (absolute path to source)
    package_dir = "/home/bl/ros2_ws/src/roboracer-autodrive/rl_control"
    tensorboard_log_dir = os.path.join(package_dir, "tensorboard_logs")
    os.makedirs(tensorboard_log_dir, exist_ok=True)

    # Checkpoint directory for saving models during training
    checkpoint_dir = os.path.join(package_dir, "checkpoints")
    os.makedirs(checkpoint_dir, exist_ok=True)
    
    run_name = f"PPO_{time.strftime('%Y%m%d_%H%M%S')}"
    
    # Check if pre-trained model exists
    model_path = os.path.join(package_dir, "models", "ppo_autodrive.zip")
    
    if os.path.exists(model_path):
        env.node.get_logger().info(f"Loading existing model from {model_path}...")
        model = PPO.load(
            model_path,
            env=env,
            verbose=1,
            device="cpu",
            tensorboard_log=tensorboard_log_dir,
            force_reset=False  # Keep current environment state
        )
    else:
        env.node.get_logger().info("No existing model found. creating new PPO model.")
        model = PPO(
            "MlpPolicy", 
            env, 
            verbose=1, 
            device="cpu", 
            tensorboard_log=tensorboard_log_dir
        )
    
    # Checkpoint callback to save model every 40000 timesteps
    checkpoint_callback = CheckpointCallback(
        save_freq=40000,
        save_path=checkpoint_dir,
        name_prefix=run_name
    )
    
    try:
        # Train the agent with callbacks
        model.learn(total_timesteps=400000, callback=checkpoint_callback, tb_log_name=run_name)
        
        # Save the final model
        model.save(os.path.join(package_dir, "models", "ppo_autodrive"))
        env.node.get_logger().info("Training finished and model saved.")
        
    except KeyboardInterrupt:
        env.node.get_logger().info("Training interrupted.")
        model.save(os.path.join(checkpoint_dir, f"{run_name}_interrupted"))
        env.node.get_logger().info(f"Model saved to {checkpoint_dir}/{run_name}_interrupted.zip")
    finally:
        env.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
