"""Train the parameter-shared Double DQN recommendation policy.

Example:
    python train.py --episodes 1500 --checkpoint checkpoints/hexagon_ddqn.pt
"""

from __future__ import annotations

import argparse
import random
from collections import deque
from dataclasses import asdict
from pathlib import Path
from typing import Deque, NamedTuple

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from formation_env import Config, SwarmFormationEnv


class Transition(NamedTuple):
    state: np.ndarray
    action: int
    reward: float
    next_state: np.ndarray
    done: float


class ReplayBuffer:
    def __init__(self, capacity: int):
        self.data: Deque[Transition] = deque(maxlen=capacity)

    def add(self, *args: object) -> None:
        self.data.append(Transition(*args))

    def sample(self, batch_size: int) -> Transition:
        batch = random.sample(self.data, batch_size)
        return Transition(*map(np.asarray, zip(*batch)))

    def __len__(self) -> int:
        return len(self.data)


class QNetwork(nn.Module):
    def __init__(self, state_dim: int, action_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 192), nn.ReLU(),
            nn.Linear(192, 192), nn.ReLU(),
            nn.Linear(192, action_dim),
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.net(state)


def epsilon_by_episode(episode: int, total: int) -> float:
    # Linear decay retains a small amount of exploration throughout training.
    #return max(0.04, 1.0 - 0.96 * episode / max(1, total * 0.75))
    return max(0.01, np.power(0.999, episode))


def choose_recommendations(env: SwarmFormationEnv, online: QNetwork, epsilon: float, device: torch.device) -> np.ndarray:
    recommendations = np.full((env.cfg.n_bots, env.cfg.n_bots), env.STOP, dtype=np.int64)
    states, pairs = [], []
    for sender in range(env.cfg.n_bots):
        for recipient in range(env.cfg.n_bots):
            if sender != recipient:
                states.append(env.state_for_recommendation(sender, recipient))
                pairs.append((sender, recipient))
    with torch.no_grad():
        values = online(torch.as_tensor(np.stack(states), dtype=torch.float32, device=device)).cpu().numpy()
    for (sender, recipient), q_values in zip(pairs, values):
        recommendations[sender, recipient] = random.randrange(env.action_dim) if random.random() < epsilon else int(q_values.argmax())
    return recommendations


def optimize(online: QNetwork, target: QNetwork, optimizer: torch.optim.Optimizer, replay: ReplayBuffer, batch_size: int, gamma: float, device: torch.device) -> float | None:
    if len(replay) < batch_size:
        return None
    batch = replay.sample(batch_size)
    states = torch.as_tensor(batch.state, dtype=torch.float32, device=device)
    actions = torch.as_tensor(batch.action, dtype=torch.int64, device=device).unsqueeze(1)
    rewards = torch.as_tensor(batch.reward, dtype=torch.float32, device=device)
    next_states = torch.as_tensor(batch.next_state, dtype=torch.float32, device=device)
    dones = torch.as_tensor(batch.done, dtype=torch.float32, device=device)
    predicted = online(states).gather(1, actions).squeeze(1)
    # Double DQN: online network selects; target network evaluates.
    with torch.no_grad():
        next_actions = online(next_states).argmax(dim=1, keepdim=True)
        next_values = target(next_states).gather(1, next_actions).squeeze(1)
        expected = rewards + gamma * (1.0 - dones) * next_values
    loss = F.smooth_l1_loss(predicted, expected)
    optimizer.zero_grad()
    loss.backward()
    torch.nn.utils.clip_grad_norm_(online.parameters(), 5.0)
    optimizer.step()
    return float(loss.item())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--checkpoint", type=Path, default=Path("checkpoints/hexagon_ddqn.pt"))
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--max-steps", type=int, default=180)
    args = parser.parse_args()
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else "cpu" if args.device == "auto" else args.device)
    env = SwarmFormationEnv(Config(max_steps=args.max_steps), seed=args.seed)
    state_dim = env.observation_dim + env.cfg.n_bots
    online, target = QNetwork(state_dim, env.action_dim).to(device), QNetwork(state_dim, env.action_dim).to(device)
    target.load_state_dict(online.state_dict())
    optimizer = torch.optim.Adam(online.parameters(), lr=3e-4)
    replay = ReplayBuffer(120_000)
    updates, successes = 0, deque(maxlen=100)
    for episode in range(1, args.episodes + 1):
        env.reset()
        epsilon, total_reward, loss = epsilon_by_episode(episode, args.episodes), 0.0, None
        info = {"success": False, "global_energy": env.global_energy()}
        for _ in range(env.cfg.max_steps):
            before_states = {(s, r): env.state_for_recommendation(s, r) for s in range(6) for r in range(6) if s != r}
            rec = choose_recommendations(env, online, epsilon, device)
            _, rewards, done, info = env.step(rec)
            for sender in range(6):
                for recipient in range(6):
                    if sender != recipient:
                        replay.add(before_states[sender, recipient], int(rec[sender, recipient]), float(rewards[sender, recipient]), env.state_for_recommendation(sender, recipient), float(done))
                        total_reward += float(rewards[sender, recipient])
            loss = optimize(online, target, optimizer, replay, batch_size=256, gamma=0.99, device=device)
            if loss is not None:
                updates += 1
                if updates % 400 == 0:
                    target.load_state_dict(online.state_dict())
            if done:
                break
        successes.append(float(info["success"]))
        if episode == 1 or episode % 25 == 0:
            print(f"episode={episode:4d} epsilon={epsilon:.3f} energy={info['global_energy']:.3f} reward={total_reward:.2f} success_100={np.mean(successes):.2%} loss={loss}")
    args.checkpoint.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"model_state": online.state_dict(), "config": asdict(env.cfg), "state_dim": state_dim, "action_dim": env.action_dim}, args.checkpoint)
    print(f"Saved checkpoint to {args.checkpoint}")


if __name__ == "__main__":
    main()
