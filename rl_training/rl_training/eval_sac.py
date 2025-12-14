# from stable_baselines3 import SAC
# from rl_training.env_sac import RoboRacerEnv
# import time

# def main():
#     env = RoboRacerEnv()

#     model = SAC.load("/home/autroboracer/ros2_ws/sac_roboracer_final.zip")

#     obs = env.reset()
#     done = False

#     start_time = time.time()

#     while not done:
#         action, _ = model.predict(obs, deterministic=True)
#         obs, reward, done, info = env.step(action)

#     lap_time = time.time() - start_time
#     print("Lap time:", lap_time)

# if __name__ == "__main__":
#     main()

from stable_baselines3 import SAC
from rl_training.env_sac import RoboRacerEnv
import time

def main():
    env = RoboRacerEnv()

    model = SAC.load("sac_roboracer_final")

    result = env.reset()
    if isinstance(result, tuple):
        obs, info = result
    else:
        obs = result

    done = False
    start_time = time.time()

    while not done:
        action, _ = model.predict(obs, deterministic=True)

        step_result = env.step(action)
        if len(step_result) == 5:
            obs, reward, terminated, truncated, info = step_result
            done = terminated or truncated
        else:
            obs, reward, done, info = step_result
    
    lap_time = time.time() - start_time
    print("Lap time:", lap_time)

    env.close()

if __name__ == "__main__":
    main()