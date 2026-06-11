from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from robo1_env import Robo1GetupEnv


def eval_pose(model, pose):
    env = Robo1GetupEnv("robo1.xml", fallen_poses=(pose,))
    obs, _ = env.reset(options={"pose": pose})

    best = {"upright": -1.0}
    final_info = {}
    for step in range(env.max_steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if info["upright"] > best["upright"]:
            best = {
                "step": step,
                "upright": info["upright"],
                "servo_angles": info["servo_angles"],
                "target": info["target"],
                "goal_pose": info["goal_pose"],
            }
        final_info = info
        if terminated or truncated:
            break

    return best, final_info


def main():
    model_path = Path("robo1_getup_ppo.zip")
    if not model_path.exists():
        raise FileNotFoundError("robo1_getup_ppo.zip is missing. Train or pretrain first.")

    env = Robo1GetupEnv("robo1.xml")
    model = PPO.load("robo1_getup_ppo", env=env, device="cpu")
    for pose in ("roll_pos", "roll_neg", "pitch_pos", "pitch_neg"):
        best, final_info = eval_pose(model, pose)
        print("pose", pose)
        print("best", best)
        print(
            "final",
            {
                "upright": final_info.get("upright"),
                "servo_angles": np.round(final_info.get("servo_angles"), 5),
                "target": np.round(final_info.get("target"), 5),
                "goal_pose": final_info.get("goal_pose"),
                "success_count": final_info.get("success_count"),
            },
        )


if __name__ == "__main__":
    main()
