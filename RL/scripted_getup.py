import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from getup_reference import getup_sequence_for_pose
from robo1_env import Robo1GetupEnv, quat_to_roll_pitch


GETUP_SEQUENCE = np.array(
    [
        [1.532655, -0.450321],
        [1.432846, 0.075935],
        [-0.570885, 1.072732],
        [-0.713748, 0.631464],
        [0.0, 0.0],
    ],
    dtype=np.float64,
)


def ramp(a, b, n):
    for i in range(n):
        t = (i + 1) / n
        yield (1.0 - t) * a + t * b


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pose",
        choices=("roll_pos", "roll_neg", "pitch_pos", "pitch_neg"),
        default="roll_pos",
    )
    args = parser.parse_args()

    env = Robo1GetupEnv("robo1.xml", fallen_poses=(args.pose,))
    env.reset(options={"pose": args.pose})
    sequence = getup_sequence_for_pose(args.pose)

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        prev = env.target.copy()
        for target in sequence:
            for ctrl in ramp(prev, target, 350):
                if not viewer.is_running():
                    return
                env.data.ctrl[:] = ctrl
                mujoco.mj_step(env.model, env.data)
                viewer.sync()
                time.sleep(env.model.opt.timestep)
            prev = target

        while viewer.is_running():
            env.data.ctrl[:] = 0.0
            mujoco.mj_step(env.model, env.data)
            viewer.sync()
            roll, pitch = quat_to_roll_pitch(env.data.qpos[3:7])
            print(
                "upright",
                round(env._upright(), 4),
                "roll",
                round(roll, 4),
                "pitch",
                round(pitch, 4),
                "servo",
                np.round(env._servo_angles(), 4),
            )
            time.sleep(env.model.opt.timestep * 20)


if __name__ == "__main__":
    main()
