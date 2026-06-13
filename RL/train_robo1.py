import argparse

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from robo1_env import Robo1GetupEnv


N_ENVS = 6


def make_env():
    def _init():
        return Robo1GetupEnv("robo1.xml")

    return _init


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=200_000)
    parser.add_argument("--n-envs", type=int, default=N_ENVS)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--model-in",
        default=None,
        help="existing PPO model zip to continue training from",
    )
    parser.add_argument(
        "--model-out",
        default="robo1_getup_ppo",
        help="output model name or zip path",
    )
    args = parser.parse_args()

    check_env(Robo1GetupEnv("robo1.xml"), warn=True)
    env = SubprocVecEnv([make_env() for _ in range(args.n_envs)])
    env.seed(args.seed)

    if args.model_in:
        model = PPO.load(args.model_in, env=env, device="cpu")
    else:
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            n_steps=512,
            batch_size=256,
            learning_rate=3e-4,
            gamma=0.99,
            seed=args.seed,
            device="cpu",
        )

    model.learn(total_timesteps=args.timesteps)
    model.save(args.model_out)
    env.close()


if __name__ == "__main__":
    main()
