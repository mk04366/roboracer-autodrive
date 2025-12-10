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
    
    # Path to models
    # Assuming we are running from the workspace or installed location, 
    # we need to find the models directory relative to the package installation or source.
    # A robust way is to check the typical location.
    
    # Try detailed paths to find the model
    possible_paths = [
        os.path.join(os.getcwd(), "src/roboracer-autodrive/rl_control/models"),
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "../models"), # When running from source
        "/home/bl/ros2_ws/src/roboracer-autodrive/rl_control/models"
    ]
    
    model_path = None
    for p in possible_paths:
        if os.path.exists(p):
            potential_model = os.path.join(p, "ppo_autodrive_model_200000_steps.zip")
            if os.path.exists(potential_model):
                model_path = potential_model
                break
    
    if model_path is None:
        env.node.get_logger().error("Could not find model file!")
        env.close()
        return

    env.node.get_logger().info(f"Loading model from: {model_path}")
    
    # Load the model
    try:
        model = PPO.load(model_path, device="cpu")
    except Exception as e:
        env.node.get_logger().error(f"Failed to load model: {e}")
        env.close()
        return

    env.node.get_logger().info("Model loaded successfully.")
    
    # Presentation Mode: Wait for user input
    print("\n" + "="*50)
    print("      ROBORACER DEMO MODE      ")
    print("      Model: PPO Autodrive     ")
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
