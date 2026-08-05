"""Train six independent Double DQNs, one for each labelled recommender bot.

The environment and reward are identical to ``train.py``.  The difference is
that sender i owns Q_i and its own replay buffer, so robot roles can specialize.

Example:
    python train_six_dqns.py --episodes 3000 \
        --checkpoint checkpoints/six_independent_ddqns.pt
"""

from __future__ import annotations

import argparse
import csv
import random
from collections import deque
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch

from formation_env import Config, SwarmFormationEnv
from train import QNetwork, ReplayBuffer, epsilon_by_episode, optimize


def choose_recommendations(
    env: SwarmFormationEnv,
    networks: list[QNetwork],
    epsilon: float,
    device: torch.device,
) -> np.ndarray:
    """Each sender uses only its own DQN to recommend actions to five peers."""
    recommendations = np.full((env.cfg.n_bots, env.cfg.n_bots), env.STOP, dtype=np.int64)
    for sender, network in enumerate(networks):
        recipients = [recipient for recipient in range(env.cfg.n_bots) if recipient != sender]
        states = np.stack([env.state_for_recommendation(sender, recipient) for recipient in recipients])
        with torch.no_grad():
            q_values = network(torch.as_tensor(states, dtype=torch.float32, device=device)).cpu().numpy()
        for recipient, values in zip(recipients, q_values):
            recommendations[sender, recipient] = random.randrange(env.action_dim) if random.random() < epsilon else int(values.argmax())
    return recommendations


def format_per_bot(values: np.ndarray) -> str:
    return "[" + ", ".join(f"{value:.3f}" if np.isfinite(value) else "nan" for value in values) + "]"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--checkpoint", type=Path, default=Path("checkpoints/six_independent_ddqns.pt"))
    parser.add_argument("--metrics", type=Path, default=Path("metrics/six_independent_ddqns.csv"))
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--max-steps", type=int, default=180)
    parser.add_argument("--log-every", type=int, default=25)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else "cpu" if args.device == "auto" else args.device)
    env = SwarmFormationEnv(Config(max_steps=args.max_steps), seed=args.seed)
    state_dim = env.observation_dim + env.cfg.n_bots
    online = [QNetwork(state_dim, env.action_dim).to(device) for _ in range(env.cfg.n_bots)]
    target = [QNetwork(state_dim, env.action_dim).to(device) for _ in range(env.cfg.n_bots)]
    for source, destination in zip(online, target):
        destination.load_state_dict(source.state_dict())
    optimizers = [torch.optim.Adam(network.parameters(), lr=3e-4) for network in online]
    replay = [ReplayBuffer(40_000) for _ in range(env.cfg.n_bots)]
    updates = np.zeros(env.cfg.n_bots, dtype=np.int64)
    successes: deque[float] = deque(maxlen=100)

    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    with args.metrics.open("w", newline="", encoding="utf-8") as metrics_file:
        fieldnames = ["episode", "epsilon", "steps", "global_energy", "success", "collision"]
        fieldnames += [f"reward_bot_{i}" for i in range(6)]
        fieldnames += [f"local_energy_bot_{i}" for i in range(6)]
        fieldnames += [f"loss_bot_{i}" for i in range(6)]
        writer = csv.DictWriter(metrics_file, fieldnames=fieldnames)
        writer.writeheader()

        for episode in range(1, args.episodes + 1):
            env.reset()
            epsilon = epsilon_by_episode(episode, args.episodes)
            episode_rewards = np.zeros(env.cfg.n_bots, dtype=np.float64)
            losses = np.full(env.cfg.n_bots, np.nan, dtype=np.float64)
            info: dict[str, object] = {"success": False, "collision": False, "global_energy": env.global_energy()}
            steps = 0

            for steps in range(1, env.cfg.max_steps + 1):
                before_states = {
                    (sender, recipient): env.state_for_recommendation(sender, recipient)
                    for sender in range(env.cfg.n_bots)
                    for recipient in range(env.cfg.n_bots)
                    if sender != recipient
                }
                recommendations = choose_recommendations(env, online, epsilon, device)
                _, rewards, done, info = env.step(recommendations)
                for sender in range(env.cfg.n_bots):
                    for recipient in range(env.cfg.n_bots):
                        if sender == recipient:
                            continue
                        replay[sender].add(
                            before_states[sender, recipient],
                            int(recommendations[sender, recipient]),
                            float(rewards[sender, recipient]),
                            env.state_for_recommendation(sender, recipient),
                            float(done),
                        )
                        episode_rewards[sender] += float(rewards[sender, recipient])

                # Each bot updates only its own Q-network from its own experience.
                for sender in range(env.cfg.n_bots):
                    loss = optimize(online[sender], target[sender], optimizers[sender], replay[sender], batch_size=256, gamma=0.99, device=device)
                    if loss is not None:
                        losses[sender] = loss
                        updates[sender] += 1
                        if updates[sender] % 400 == 0:
                            target[sender].load_state_dict(online[sender].state_dict())
                if done:
                    break

            local_energy = np.asarray(info["local_energy"], dtype=np.float64)
            successes.append(float(info["success"]))
            row: dict[str, float | int | bool] = {
                "episode": episode,
                "epsilon": epsilon,
                "steps": steps,
                "global_energy": float(info["global_energy"]),
                "success": bool(info["success"]),
                "collision": bool(info["collision"]),
            }
            row.update({f"reward_bot_{i}": float(episode_rewards[i]) for i in range(6)})
            row.update({f"local_energy_bot_{i}": float(local_energy[i]) for i in range(6)})
            row.update({f"loss_bot_{i}": float(losses[i]) for i in range(6)})
            writer.writerow(row)
            metrics_file.flush()

            if episode == 1 or episode % args.log_every == 0:
                print(
                    f"episode={episode:4d} eps={epsilon:.3f} steps={steps:3d} "
                    f"global_E={float(info['global_energy']):.3f} success_100={np.mean(successes):.2%} "
                    f"collision={bool(info['collision'])}\n"
                    f"  rewards={format_per_bot(episode_rewards)}\n"
                    f"  local_E={format_per_bot(local_energy)}\n"
                    f"  losses ={format_per_bot(losses)}"
                )

    args.checkpoint.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_states": [network.state_dict() for network in online],
            "config": asdict(env.cfg),
            "state_dim": state_dim,
            "action_dim": env.action_dim,
            "architecture": "six_independent_sender_ddqns",
        },
        args.checkpoint,
    )
    print(f"Saved six-network checkpoint to {args.checkpoint}")
    print(f"Saved episode metrics to {args.metrics}")


if __name__ == "__main__":
    main()
