"""Run and animate a trained swarm from a random initial configuration.

Example:
    python simulate.py --checkpoint checkpoints/hexagon_ddqn.pt --seed 42
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import torch

from formation_env import Config, SwarmFormationEnv
from train import QNetwork, choose_recommendations


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--steps", type=int, default=180)
    parser.add_argument("--save", type=Path, help="Optional output GIF path (requires Pillow).")
    args = parser.parse_args()
    device = torch.device("cpu")
    saved = torch.load(args.checkpoint, map_location=device, weights_only=False)
    env = SwarmFormationEnv(Config(**saved["config"]), seed=args.seed)
    model = QNetwork(saved["state_dim"], saved["action_dim"]).to(device)
    model.load_state_dict(saved["model_state"]); model.eval()
    env.reset(seed=args.seed)
    frames, energies = [env.positions.copy()], [env.global_energy()]
    info = {"success": False}
    for _ in range(min(args.steps, env.cfg.max_steps)):
        rec = choose_recommendations(env, model, epsilon=0.0, device=device)
        _, _, done, info = env.step(rec)
        frames.append(env.positions.copy()); energies.append(float(info["global_energy"]))
        if done:
            break

    fig, (ax, energy_ax) = plt.subplots(1, 2, figsize=(11, 5))
    ax.set(xlim=(-env.cfg.arena_half_extent, env.cfg.arena_half_extent), ylim=(-env.cfg.arena_half_extent, env.cfg.arena_half_extent), aspect="equal", title="Six-robot formation rollout")
    ax.scatter(env.target_positions[:, 0], env.target_positions[:, 1], marker="x", s=100, c="black", label="labelled target shape")
    dots = ax.scatter([], [], s=90, c=np.arange(6), cmap="tab10", vmin=0, vmax=9, label="robots")
    labels = [ax.text(0, 0, str(i), ha="center", va="center", color="white", fontsize=8) for i in range(6)]
    ax.legend(loc="upper right")
    energy_ax.set(xlim=(0, max(1, len(energies) - 1)), ylim=(0, max(1.0, max(energies) * 1.05)), xlabel="control cycle", ylabel="global angular energy", title="Formation energy")
    energy_line, = energy_ax.plot([], [], color="tab:purple")

    def update(frame: int):
        dots.set_offsets(frames[frame])
        for i, label in enumerate(labels):
            label.set_position(frames[frame][i])
        energy_line.set_data(np.arange(frame + 1), energies[: frame + 1])
        ax.set_title(f"Six-robot formation rollout — step {frame}, energy {energies[frame]:.3f}")
        return [dots, energy_line, *labels]

    movie = animation.FuncAnimation(fig, update, frames=len(frames), interval=90, blit=False, repeat=False)
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        movie.save(args.save, writer="pillow", fps=12)
        print(f"Saved animation to {args.save}")
    print(f"Finished after {len(frames)-1} steps: energy={energies[-1]:.4f}, success={info['success']}")
    plt.show()


if __name__ == "__main__":
    main()
