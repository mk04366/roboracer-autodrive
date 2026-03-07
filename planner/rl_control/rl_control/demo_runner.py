import rclpy
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
import os
import sys
import time
from rl_control.rl_agent_node import AutodriveEnv

def main(args=None):
    rclpy.init(args=args)
    
    # Create the environment
    # Using the existing AutodriveEnv from rl_agent_node.py
    env = AutodriveEnv(node_name='rl_demo_node')
    
    # Model directory (absolute path to source)
    model_dir = "/home/bl/ros2_ws/src/roboracer-autodrive/planner/rl_control/models"
    model_name = "ppo_autodrive_wp.zip"
    # model_name = "rl_general_model.zip"
    model_path = os.path.join(model_dir, model_name)
    
    # Check if model exists
    if not os.path.exists(model_path):
        env.node.get_logger().error(f"Model not found at: {model_path}")
        env.node.get_logger().info(f"Available models in {model_dir}:")
        if os.path.exists(model_dir):
            for f in os.listdir(model_dir):
                env.node.get_logger().info(f"  - {f}")
        env.close()
        rclpy.shutdown()
        return

    env.node.get_logger().info(f"Loading model from: {model_path}")
    
    # Load the model
    try:
        model = PPO.load(model_path, device="cpu")
    except Exception as e:
        env.node.get_logger().error(f"Failed to load model: {e}")
        env.close()
        rclpy.shutdown()
        return

    env.node.get_logger().info("Model loaded successfully.")
    
    # Presentation Mode: Wait for user input
    print("\n" + "="*50)
    print("      ROBORACER DEMO MODE      ")
    print(f"      Model: {model_name}     ")
    print("      Deterministic: ON        ")
    print("="*50)
    input("\n>>> Press ENTER to start the car... <<<")
    print("Starting...")

    obs, _ = env.reset()
    
    try:
        while True:
            # Predict action with deterministic=True for best performance
            action, _states = model.predict(obs, deterministic=True)
            
            # Step the environment
            obs, reward, terminated, truncated, info = env.step(action)
            
            if terminated or truncated:
                env.node.get_logger().warn("Episode finished. Resetting...")
                obs, _ = env.reset()
                
    except KeyboardInterrupt:
        env.node.get_logger().info("Demo stopped by user.")
    finally:
        env.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
