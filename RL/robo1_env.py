import math
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces


FALLEN_POSES = {
    "roll_pos": (math.pi / 2.0, 0.0),
    "roll_neg": (-math.pi / 2.0, 0.0),
    "pitch_pos": (0.0, math.pi / 2.0),
    "pitch_neg": (0.0, -math.pi / 2.0),
}
DEFAULT_FALLEN_POSES = tuple(FALLEN_POSES.keys())


def roll_pitch_to_quat(roll, pitch):
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp], dtype=np.float64)


def quat_to_roll_pitch(q):
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


class Robo1GetupEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, xml_path="robo1.xml", fallen_poses=None):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(str(Path(xml_path)))
        self.data = mujoco.MjData(self.model)

        self.frame_skip = 20
        self.max_steps = 700
        self.target_delta = 0.08
        self.target_limit = 1.55
        self.settle_steps = 200
        self.fallen_poses = list(fallen_poses or DEFAULT_FALLEN_POSES)
        self.current_pose_name = self.fallen_poses[0]

        self.foot_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "foot"
        )
        self.servo1_qpos_id = self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo1_joint")
        ]
        self.servo2_qpos_id = self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo2_joint")
        ]
        self.servo1_qvel_id = self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo1_joint")
        ]
        self.servo2_qvel_id = self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo2_joint")
        ]
        self.standing_height = self._compute_standing_height()

        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=np.array(
                [-math.pi, -math.pi, -1.55, -1.55],
                dtype=np.float32,
            ),
            high=np.array(
                [math.pi, math.pi, 1.55, 1.55],
                dtype=np.float32,
            ),
            dtype=np.float32,
        )

        self.step_count = 0
        self.target = np.zeros(2, dtype=np.float64)
        self.success_count = 0
        self.prev_upright = 0.0

    def _compute_standing_height(self):
        data = mujoco.MjData(self.model)
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qpos[0:3] = [0.0, 0.0, 0.08]
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        data.qpos[self.servo1_qpos_id] = 0.0
        data.qpos[self.servo2_qpos_id] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, data)

        for _ in range(self.settle_steps):
            data.ctrl[:] = 0.0
            mujoco.mj_step(self.model, data)

        return float(data.xpos[self.foot_body_id, 2])

    def _set_fixed_fallen_pose(self, pose_name=None):
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0

        self.data.qpos[0:3] = [0.0, 0.0, 0.08]
        self.current_pose_name = pose_name or self.current_pose_name
        roll, pitch = FALLEN_POSES[self.current_pose_name]
        self.data.qpos[3:7] = roll_pitch_to_quat(roll, pitch)
        self.data.qpos[self.servo1_qpos_id] = 0.0
        self.data.qpos[self.servo2_qpos_id] = 0.0

        self.target[:] = 0.0
        self.data.ctrl[:] = self.target
        mujoco.mj_forward(self.model, self.data)

        for _ in range(self.settle_steps):
            self.data.ctrl[:] = self.target
            mujoco.mj_step(self.model, self.data)

        self.target[:] = 0.0
        self.data.ctrl[:] = self.target

    def _get_obs(self):
        roll, pitch = quat_to_roll_pitch(self.data.qpos[3:7])
        return np.array(
            [
                roll,
                pitch,
                self.target[0],
                self.target[1],
            ],
            dtype=np.float32,
        )

    def _upright(self):
        xmat = self.data.xmat[self.foot_body_id].reshape(3, 3)
        return float(xmat[2, 2])

    def _foot_height(self):
        return float(self.data.xpos[self.foot_body_id, 2])

    def _servo_angles(self):
        return np.array(
            [
                self.data.qpos[self.servo1_qpos_id],
                self.data.qpos[self.servo2_qpos_id],
            ],
            dtype=np.float64,
        )

    def _servo_velocities(self):
        return np.array(
            [
                self.data.qvel[self.servo1_qvel_id],
                self.data.qvel[self.servo2_qvel_id],
            ],
            dtype=np.float64,
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.success_count = 0
        pose_name = None
        if options is not None:
            pose_name = options.get("pose")
        if pose_name is None:
            pose_name = self.np_random.choice(self.fallen_poses)
        self._set_fixed_fallen_pose(pose_name)
        self.prev_upright = self._upright()
        return self._get_obs(), {"pose": self.current_pose_name}

    def step(self, action):
        action = np.asarray(action, dtype=np.float64)
        action = np.clip(action, -1.0, 1.0)

        old_target = self.target.copy()
        self.target += action * self.target_delta
        self.target = np.clip(self.target, -self.target_limit, self.target_limit)

        for _ in range(self.frame_skip):
            self.data.ctrl[:] = self.target
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        obs = self._get_obs()

        upright = self._upright()
        upright_progress = upright - self.prev_upright
        self.prev_upright = upright
        roll, pitch = float(obs[0]), float(obs[1])
        servo_angles = self._servo_angles()
        servo_velocities = self._servo_velocities()
        foot_height_error = self._foot_height() - self.standing_height

        tilt_cost = 0.2 * (roll * roll + pitch * pitch)
        stand_gate = float(np.clip((upright - 0.65) / 0.35, 0.0, 1.0))
        servo_zero_cost = stand_gate * 0.5 * float(np.sum(servo_angles * servo_angles))
        target_zero_cost = stand_gate * 0.2 * float(np.sum(self.target * self.target))
        height_cost = stand_gate * 5.0 * foot_height_error * foot_height_error
        body_velocity_cost = 0.01 * float(np.sum(self.data.qvel[0:6] * self.data.qvel[0:6]))
        servo_velocity_cost = 0.005 * float(np.sum(servo_velocities * servo_velocities))
        action_cost = 0.005 * float(np.sum(action * action))
        servo_motion_cost = 0.02 * float(np.sum((self.target - old_target) ** 2))
        reward = (
            2.0 * upright
            + 8.0 * upright_progress
            - tilt_cost
            - servo_zero_cost
            - target_zero_cost
            - height_cost
            - body_velocity_cost
            - servo_velocity_cost
            - action_cost
            - servo_motion_cost
        )

        goal_pose = (
            upright > 0.95
            and abs(roll) < 0.25
            and abs(pitch) < 0.25
            and np.max(np.abs(servo_angles)) < 0.12
            and abs(foot_height_error) < 0.02
            and np.linalg.norm(self.data.qvel[0:6]) < 0.25
        )

        if goal_pose:
            self.success_count += 1
            reward += 5.0
        else:
            self.success_count = 0

        terminated = self.success_count >= 50
        truncated = self.step_count >= self.max_steps
        info = {
            "pose": self.current_pose_name,
            "upright": upright,
            "foot_height_error": foot_height_error,
            "servo_angles": servo_angles.copy(),
            "goal_pose": goal_pose,
            "target": self.target.copy(),
            "success_count": self.success_count,
        }
        return obs, reward, terminated, truncated, info
