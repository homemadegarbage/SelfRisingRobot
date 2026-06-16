import argparse

import numpy as np


FALLEN_POSES = ("roll_pos", "roll_neg", "pitch_pos", "pitch_neg")


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


def eval_sequence(pose, sequence):
    from robo1_env import Robo1GetupEnv

    env = Robo1GetupEnv("robo1.xml", fallen_poses=(pose,))
    obs, _ = env.reset(options={"pose": pose})
    best = {"score": -1e9}
    final = {}

    for waypoint in expert_target_schedule(env, sequence):
        obs, _, terminated, truncated, info = env.step(scripted_action(env, waypoint))
        servo = info["servo_angles"]
        score = (
            info["upright"]
            - 0.05 * float(np.sum(servo * servo))
            - 0.02 * float(np.sum(info["target"] * info["target"]))
        )
        if score > best["score"]:
            best = {
                "score": score,
                "upright": info["upright"],
                "servo_angles": servo.copy(),
                "target": info["target"].copy(),
                "goal_pose": info["goal_pose"],
            }
        final = info
        if terminated or truncated:
            break

    return best, final


def build_candidates(seed, random_count):
    rng = np.random.default_rng(seed)
    base_vals = [-1.55, -1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2, 1.55]
    candidates = []

    for a in base_vals:
        for b in base_vals:
            candidates.append(np.array([[a, b], [0.0, b], [0.0, 0.0]], dtype=np.float64))
            candidates.append(np.array([[a, b], [a, 0.0], [0.0, 0.0]], dtype=np.float64))
            candidates.append(np.array([[0.0, b], [a, b], [0.0, 0.0]], dtype=np.float64))

    for _ in range(random_count):
        seq = rng.uniform(-1.55, 1.55, size=(5, 2))
        seq[-1] = 0.0
        candidates.append(seq)

    return candidates


def print_reference_block(results):
    print()
    print("REFERENCE_CANDIDATES")
    for pose, seq in results.items():
        print(pose)
        print(np.array2string(seq, precision=6, separator=", "))


def main():
    parser = argparse.ArgumentParser(
        description="Search scripted get-up target-angle sequences for each fallen pose."
    )
    parser.add_argument(
        "--pose",
        choices=FALLEN_POSES,
        action="append",
        help="pose to search; can be passed multiple times. Defaults to all poses.",
    )
    parser.add_argument("--seed", type=int, default=4)
    parser.add_argument("--random-count", type=int, default=1200)
    parser.add_argument(
        "--no-stop-on-goal",
        action="store_true",
        help="keep searching even after a candidate reaches goal_pose",
    )
    args = parser.parse_args()

    poses = tuple(args.pose) if args.pose else FALLEN_POSES
    candidates = build_candidates(args.seed, args.random_count)
    results = {}

    for pose in poses:
        best = None
        best_seq = None
        print("POSE", pose)
        for i, seq in enumerate(candidates):
            detail, final = eval_sequence(pose, seq)
            if best is None or detail["score"] > best["score"]:
                best = detail
                best_seq = seq
                print(
                    "new_best",
                    i,
                    "score",
                    round(best["score"], 4),
                    "upright",
                    round(best["upright"], 4),
                    "goal",
                    best["goal_pose"],
                    "servo",
                    np.round(best["servo_angles"], 4),
                    "target",
                    np.round(best["target"], 4),
                    "seq",
                    np.round(best_seq, 4),
                    "final_up",
                    round(final["upright"], 4),
                )
                if best["goal_pose"] and not args.no_stop_on_goal:
                    break

        print("BEST_SEQ", pose, np.round(best_seq, 6))
        print("BEST", best)
        results[pose] = best_seq

    print_reference_block(results)


if __name__ == "__main__":
    main()
