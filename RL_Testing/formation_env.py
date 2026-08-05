"""Six-robot decentralized formation environment.

The environment follows the project design closely:
* a robot observes bearings to the other labelled robots plus all headings;
* every sender recommends one primitive for every *other* recipient;
* each recipient sums its recommendations and executes the closest primitive;
* formation quality is measured with the angular local energy described in
  ``RL_Solution.md``.

Positions are used internally only to simulate motion and derive bearings;
they are not exposed to the policy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np


@dataclass(frozen=True)
class Config:
    n_bots: int = 6
    # SI units.  The formation objective remains scale-free; these values only
    # set a sensible physical simulation scale for 0.5 cm / 1 cm actions.
    arena_half_extent: float = 0.30
    min_separation: float = 0.08
    short_step: float = 0.005
    long_step: float = 0.010
    max_steps: int = 180
    success_energy: float = 0.08


class SwarmFormationEnv:
    """A labelled six-robot planar formation task.

    The thirteen primitives are STOP plus six equally spaced body-frame
    movement vectors at 0.5 cm and the same six vectors at 1 cm.
    Recommendations are represented in the *recipient's* body frame before
    being summed.
    """

    STOP = 0
    ACTION_NAMES = (
        "stop",
        "short_0deg", "short_60deg", "short_120deg", "short_180deg", "short_240deg", "short_300deg",
        "long_0deg", "long_60deg", "long_120deg", "long_180deg", "long_240deg", "long_300deg",
    )

    def __init__(self, config: Config | None = None, seed: int | None = None):
        self.cfg = config or Config()
        if self.cfg.n_bots != 6:
            raise ValueError("This implementation is intentionally configured for six robots.")
        self.rng = np.random.default_rng(seed)
        self.positions = np.zeros((self.cfg.n_bots, 2), dtype=np.float32)
        self.headings = np.zeros(self.cfg.n_bots, dtype=np.float32)
        self.step_count = 0
        self.target_positions = self._regular_hexagon(radius=0.13)
        self._target_local_angles = self._all_local_angles(self.target_positions)

    def _regular_hexagon(self, radius: float) -> np.ndarray:
        angles = np.arange(self.cfg.n_bots) * (2.0 * np.pi / self.cfg.n_bots)
        return np.stack((radius * np.cos(angles), radius * np.sin(angles)), axis=1).astype(np.float32)

    def reset(self, seed: int | None = None) -> Dict[str, np.ndarray]:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.step_count = 0
        # Rejection sampling avoids nearly coincident robots, where bearings are ill-defined.
        samples: List[np.ndarray] = []
        while len(samples) < self.cfg.n_bots:
            candidate = self.rng.uniform(-0.23, 0.23, size=2).astype(np.float32)
            if all(np.linalg.norm(candidate - old) >= self.cfg.min_separation * 1.7 for old in samples):
                samples.append(candidate)
        self.positions = np.asarray(samples, dtype=np.float32)
        self.headings = self.rng.uniform(-np.pi, np.pi, size=self.cfg.n_bots).astype(np.float32)
        return self.observations()

    @staticmethod
    def _wrap(angle: np.ndarray | float) -> np.ndarray | float:
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def _all_local_angles(self, positions: np.ndarray) -> np.ndarray:
        """bearing[i, j] is the global bearing from i to j (i != j)."""
        delta = positions[None, :, :] - positions[:, None, :]
        angles = np.arctan2(delta[:, :, 1], delta[:, :, 0])
        np.fill_diagonal(angles, 0.0)
        return angles

    def local_energy(self, robot: int, positions: np.ndarray | None = None) -> float:
        """Angular energy relative to the first labelled peer, as in the notes."""
        pos = self.positions if positions is None else positions
        angles = self._all_local_angles(pos)[robot]
        target = self._target_local_angles[robot]
        peers = [j for j in range(self.cfg.n_bots) if j != robot]
        ref = peers[0]
        energy = 0.0
        for j in peers[1:]:
            actual_alpha = self._wrap(angles[j] - angles[ref])
            required_alpha = self._wrap(target[j] - target[ref])
            energy += 1.0 - np.cos(actual_alpha - required_alpha)
        return float(energy)

    def global_energy(self, positions: np.ndarray | None = None) -> float:
        return float(sum(self.local_energy(i, positions) for i in range(self.cfg.n_bots)))

    def min_pair_distance(self, positions: np.ndarray | None = None) -> float:
        pos = self.positions if positions is None else positions
        delta = pos[None, :, :] - pos[:, None, :]
        distances = np.linalg.norm(delta, axis=-1)
        distances += np.eye(self.cfg.n_bots) * 1e6
        return float(distances.min())

    def observation(self, sender: int) -> np.ndarray:
        """5 relative bearings (sin/cos) + all 6 global headings (sin/cos)."""
        angles = self._all_local_angles(self.positions)[sender]
        peers = [j for j in range(self.cfg.n_bots) if j != sender]
        relative_bearings = self._wrap(angles[peers] - self.headings[sender])
        return np.concatenate(
            [np.sin(relative_bearings), np.cos(relative_bearings), np.sin(self.headings), np.cos(self.headings)]
        ).astype(np.float32)

    def observations(self) -> Dict[str, np.ndarray]:
        return {"agents": np.stack([self.observation(i) for i in range(self.cfg.n_bots)]).astype(np.float32)}

    @property
    def observation_dim(self) -> int:
        return 22

    @property
    def action_dim(self) -> int:
        return len(self.ACTION_NAMES)

    def recipient_features(self, recipient: int) -> np.ndarray:
        one_hot = np.zeros(self.cfg.n_bots, dtype=np.float32)
        one_hot[recipient] = 1.0
        return one_hot

    def state_for_recommendation(self, sender: int, recipient: int) -> np.ndarray:
        return np.concatenate((self.observation(sender), self.recipient_features(recipient))).astype(np.float32)

    def _primitive(self, action: int) -> np.ndarray:
        """Return one requested displacement vector in the recipient body frame."""
        if action == self.STOP:
            return np.zeros(2, dtype=np.float32)
        if 1 <= action <= 6:
            length, direction_index = self.cfg.short_step, action - 1
        elif 7 <= action <= 12:
            length, direction_index = self.cfg.long_step, action - 7
        else:
            raise ValueError(f"Unknown action {action}")
        angle = direction_index * np.pi / 3.0
        return np.array([length * np.cos(angle), length * np.sin(angle)], dtype=np.float32)

    def _project_to_primitive(self, body_vector_sum: np.ndarray) -> int:
        """Choose the discrete movement vector closest to the summed request."""
        best_action, best_cost = self.STOP, float("inf")
        for action in range(self.action_dim):
            candidate = self._primitive(action)
            cost = float(np.sum((body_vector_sum - candidate) ** 2))
            if cost < best_cost:
                best_action, best_cost = action, cost
        return best_action

    @staticmethod
    def _body_to_world(body_vector: np.ndarray, heading: float) -> np.ndarray:
        """Rotate a local displacement into the global simulation frame."""
        c, s = np.cos(heading), np.sin(heading)
        return np.array([c * body_vector[0] - s * body_vector[1], s * body_vector[0] + c * body_vector[1]], dtype=np.float32)

    def _executed_actions(self, recommendations: np.ndarray) -> np.ndarray:
        """Aggregate incoming recommendations and project one command per robot."""
        executed = np.full(self.cfg.n_bots, self.STOP, dtype=np.int64)
        for recipient in range(self.cfg.n_bots):
            actions = [int(recommendations[sender, recipient]) for sender in range(self.cfg.n_bots) if sender != recipient]
            body_vector_sum = np.asarray([self._primitive(action) for action in actions], dtype=np.float32).sum(axis=0)
            executed[recipient] = self._project_to_primitive(body_vector_sum)
        return executed

    def _positions_after_executed(self, positions: np.ndarray, headings: np.ndarray, executed: np.ndarray) -> np.ndarray:
        """Apply one projected movement vector to each robot from a shared state."""
        next_positions = positions.copy()
        for recipient, action in enumerate(executed):
            next_positions[recipient] += self._body_to_world(self._primitive(int(action)), headings[recipient])
        return np.clip(next_positions, -self.cfg.arena_half_extent, self.cfg.arena_half_extent)

    def step(self, recommendations: np.ndarray) -> Tuple[Dict[str, np.ndarray], np.ndarray, bool, Dict[str, object]]:
        """Advance one collective control cycle.

        ``recommendations[sender, recipient]`` is an integer primitive.  The
        diagonal is ignored because robots do not recommend their own motion.
        Returns a reward matrix indexed by the same sender/recipient pairs.
        """
        rec = np.asarray(recommendations, dtype=np.int64)
        if rec.shape != (self.cfg.n_bots, self.cfg.n_bots):
            raise ValueError("recommendations must have shape (6, 6)")
        if np.any((rec < 0) | (rec >= self.action_dim)):
            raise ValueError("recommendations contain an invalid action")

        old_pos, old_headings = self.positions.copy(), self.headings.copy()
        executed = self._executed_actions(rec)
        aggregate_positions = self._positions_after_executed(old_pos, old_headings, executed)

        # Marginal local credit under the real controller: compare the sender's
        # local energy with all recommendations against the same aggregate with
        # just its recommendation to this recipient removed.
        rewards = np.zeros_like(rec, dtype=np.float32)
        for sender in range(self.cfg.n_bots):
            for recipient in range(self.cfg.n_bots):
                if sender == recipient:
                    continue
                without_one = rec.copy()
                without_one[sender, recipient] = self.STOP
                without_executed = self._executed_actions(without_one)
                without_positions = self._positions_after_executed(old_pos, old_headings, without_executed)
                rewards[sender, recipient] = self.local_energy(sender, without_positions) - self.local_energy(sender, aggregate_positions)

        self.positions = aggregate_positions
        self.step_count += 1

        new_global = self.global_energy()
        collision = self.min_pair_distance() < self.cfg.min_separation
        # Each recommender learns only to reduce its own angular energy; no
        # global-energy reward or distance-based repulsion term is used.
        if collision:
            rewards -= 0.30
        np.fill_diagonal(rewards, 0.0)
        success = new_global < self.cfg.success_energy and not collision
        done = bool(success or collision or self.step_count >= self.cfg.max_steps)
        info: Dict[str, object] = {
            "global_energy": new_global,
            "local_energy": np.array([self.local_energy(i) for i in range(self.cfg.n_bots)]),
            "executed_actions": executed,
            "collision": collision,
            "success": success,
            "min_pair_distance": self.min_pair_distance(),
        }
        return self.observations(), rewards, done, info
