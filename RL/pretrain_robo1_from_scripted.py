import numpy as np
import torch
import mujoco
from stable_baselines3 import PPO

from getup_reference import getup_sequence_for_pose
from robo1_env import FALLEN_POSES, Robo1GetupEnv, roll_pitch_to_quat


POSE_VARIANTS_DEG = {
    "roll_pos": [(0.0, 0.0)],
    "roll_neg": [(0.0, 0.0)],
    "pitch_pos": [
        (0.0, 0.0),
        (-20.0, -20.0),
        (-20.0, 0.0),
        (-20.0, 20.0),
        (-10.0, -10.0),
        (-10.0, 10.0),
        (0.0, -20.0),
        (0.0, 20.0),
        (10.0, -10.0),
        (10.0, 10.0),
        (20.0, -20.0),
        (20.0, 0.0),
        (20.0, 20.0),
    ],
    "pitch_neg": [(0.0, 0.0)],
}


def expert_target_schedule(env, sequence):
    targets = []
    prev = np.zeros(2, dtype=np.float64)
    for waypoint in sequence:
        for i in range(350):
            t = (i + 1) / 350
            ctrl = (1.0 - t) * prev + t * waypoint
            if i % env.frame_skip == env.frame_skip - 1:
                targets.append(ctrl.copy())
        prev = waypoint

    for _ in range(150):
        targets.append(np.zeros(2, dtype=np.float64))
    return targets


def scripted_action(env, waypoint):
    error = waypoint - env.target
    return np.clip(error / env.target_delta, -1.0, 1.0).astype(np.float32)


def reset_variant(env, pose, roll_offset_deg, pitch_offset_deg):
    base_roll, base_pitch = FALLEN_POSES[pose]
    roll = base_roll + np.deg2rad(roll_offset_deg)
    pitch = base_pitch + np.deg2rad(pitch_offset_deg)

    env.data.qpos[:] = 0.0
    env.data.qvel[:] = 0.0
    env.data.qpos[0:3] = [0.0, 0.0, 0.08]
    env.data.qpos[3:7] = roll_pitch_to_quat(roll, pitch)
    env.target[:] = 0.0
    env.data.ctrl[:] = 0.0
    mujoco.mj_forward(env.model, env.data)

    for _ in range(env.settle_steps):
        env.data.ctrl[:] = 0.0
        mujoco.mj_step(env.model, env.data)

    env.target[:] = 0.0
    env.data.ctrl[:] = 0.0
    env.step_count = 0
    env.success_count = 0
    env.prev_upright = env._upright()
    env.current_pose_name = pose
    return env._get_obs(), {"pose": pose}


def collect_expert_data():
    observations = []
    actions = []

    for pose in ("roll_pos", "roll_neg", "pitch_pos", "pitch_neg"):
        for roll_offset, pitch_offset in POSE_VARIANTS_DEG[pose]:
            env = Robo1GetupEnv("robo1.xml", fallen_poses=(pose,))
            obs, _ = reset_variant(env, pose, roll_offset, pitch_offset)
            sequence = getup_sequence_for_pose(pose)

            for waypoint in expert_target_schedule(env, sequence):
                action = scripted_action(env, waypoint)
                observations.append(obs.copy())
                actions.append(action.copy())

                obs, _, terminated, truncated, _ = env.step(action)
                if terminated or truncated:
                    break

    return np.asarray(observations, dtype=np.float32), np.asarray(actions, dtype=np.float32)


def evaluate_policy(model):
    results = {}
    for pose in ("roll_pos", "roll_neg", "pitch_pos", "pitch_neg"):
        env = Robo1GetupEnv("robo1.xml", fallen_poses=(pose,))
        obs, _ = env.reset(options={"pose": pose})
        best_upright = -1.0
        final_info = {}

        for _ in range(env.max_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            best_upright = max(best_upright, info["upright"])
            final_info = info
            if terminated or truncated:
                break

        results[pose] = {"best_upright": best_upright, "final_info": final_info}
    return results


def main():
    observations, actions = collect_expert_data()
    env = Robo1GetupEnv("robo1.xml")
    model = PPO(
        "MlpPolicy",
        env,
        n_steps=512,
        batch_size=256,
        learning_rate=3e-4,
        gamma=0.99,
        verbose=0,
        device="cpu",
    )

    obs_tensor = torch.as_tensor(observations, dtype=torch.float32, device=model.device)
    act_tensor = torch.as_tensor(actions, dtype=torch.float32, device=model.device)
    optimizer = torch.optim.Adam(model.policy.parameters(), lr=1e-3)

    for epoch in range(5000):
        dist = model.policy.get_distribution(obs_tensor)
        pred = dist.distribution.mean
        loss = torch.mean((pred - act_tensor) ** 2)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if epoch % 500 == 0:
            print("epoch", epoch, "loss", float(loss.detach().cpu()))

    print("expert_samples", len(observations))
    results = evaluate_policy(model)
    for pose, result in results.items():
        print(pose, result)
    model.save("robo1_getup_ppo")


if __name__ == "__main__":
    main()
