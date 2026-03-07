"""
Resume training script for rl_general.py

This script loads a previously saved/interrupted model checkpoint
and continues training from where it left off.

Usage:
    ros2 run rl_control rl_general_resume
"""

import rclpy
import os
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback

# Import the environment from the original training script
from rl_control.rl_general import AutodriveEnv


def main(args=None):
    rclpy.init(args=args)
    
    # Setup paths
    package_dir = "/home/bl/ros2_ws/src/roboracer-autodrive/rl_control"
    checkpoint_dir = os.path.join(package_dir, "checkpoints")
    # Continue logging to the existing TensorBoard run
    log_dir = os.path.join(package_dir, "tensorboard_logs", "PPO_1")
    
    # Path to the interrupted model checkpoint
    model_path = os.path.join(checkpoint_dir, "rl_general_model_interrupted.zip")
    
    # Create the environment
    env = AutodriveEnv()
    
    print(f"Loading model from: {model_path}")
    print(f"TensorBoard logs will continue in: {log_dir}")
    
    # Load the saved model and set the environment
    # Set tensorboard_log to parent dir, but we'll use tb_log_name to continue in PPO_1
    model = PPO.load(model_path, env=env, device="cpu", tensorboard_log=os.path.join(package_dir, "tensorboard_logs"))
    
    print("Model loaded successfully!")
    print(f"Continuing training with TensorBoard logs at: {log_dir}")
    
    # Create checkpoint callback (same settings as original)
    checkpoint_callback = CheckpointCallback(
        save_freq=50000,
        save_path=checkpoint_dir,
        name_prefix="rl_general_model"
    )
    
    # Configure remaining timesteps
    # You can adjust this value based on how many more steps you want to train
    REMAINING_TIMESTEPS = 700000  # Default: train for 700k more steps
    
    try:
        # Continue training
        print(f"Starting training for {REMAINING_TIMESTEPS} timesteps...")
        model.learn(
            total_timesteps=REMAINING_TIMESTEPS, 
            callback=checkpoint_callback,
            reset_num_timesteps=False  # Continue counting from previous timesteps
        )
        
        # Save the final model
        final_model_path = os.path.join(checkpoint_dir, "rl_general_model_final")
        model.save(final_model_path)
        env.node.get_logger().info(f"Training finished! Model saved to {final_model_path}.zip")
        
    except KeyboardInterrupt:
        env.node.get_logger().info("Training interrupted again.")
        model.save(os.path.join(checkpoint_dir, "rl_general_model_interrupted"))
        env.node.get_logger().info(f"Model saved to {checkpoint_dir}/rl_general_model_interrupted.zip")
    finally:
        env.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
