from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from rl_training.env_sac import RoboRacerEnv


def main():
    env = RoboRacerEnv()

    model = SAC(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        buffer_size=300000,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        train_freq=1,
        gradient_steps=1,
    )

    checkpoint = CheckpointCallback(
        save_freq=10000,
        save_path="./checkpoints/",
        name_prefix="sac_roboracer"
    )

    model.learn(
        total_timesteps=500000,
        callback=checkpoint
    )

    model.save("sac_roboracer_final")
    env.close()


if __name__ == "__main__":
    main()