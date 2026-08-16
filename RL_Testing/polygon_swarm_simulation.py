"""Arbitrary-size regular-polygon swarm simulation.

This is a dimension-general version of ``SimulationOfSwarmAlg.ipynb``.  It
keeps its local angular energy, inverse-distance penalty, finite-difference
gradient, and synchronous gradient update intact.  The only simulation-model
additions are the requested sensor and actuator errors.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


MOVEMENT_ERROR_STD = 0.02
"""Gaussian standard deviation of the additive random-walk movement drift."""

BEARING_ERROR_STD = 0.05
"""Gaussian standard deviation of the multiplicative bearing measurement error."""


@dataclass
class SimulationResult:
    """State history and stopping information returned by :func:`simulate_polygon`."""

    positions: np.ndarray
    energies: np.ndarray
    gradient_norms: np.ndarray
    converged: bool
    energy_threshold: float


def get_angle(v1: np.ndarray, v2: np.ndarray, v3: np.ndarray) -> float:
    """Signed angle between the rays from ``v1`` to ``v2`` and ``v3``."""
    left, right = v1 - v2, v1 - v3
    return float(np.arctan2(left[0] * right[1] - left[1] * right[0], np.dot(left, right)))


def get_energy(vec: np.ndarray, reqd_angles: np.ndarray) -> float:
    """The notebook's local energy, generalized from six to ``len(vec)`` bots."""
    if len(vec) < 3:
        raise ValueError("At least three bots are required to form a polygon.")
    angles = np.array([get_angle(vec[0], vec[1], vec[k]) for k in range(2, len(vec))])
    distances = np.linalg.norm(vec[1:] - vec[0], axis=1)
    # The small floor only avoids undefined numerical division at an exact overlap.
    return float(
        len(angles)
        - np.cos(angles - reqd_angles).sum()
        + 0.01 * np.reciprocal(np.maximum(distances, 1e-12)).sum()
    )


def get_new_grads(
    j: int,
    angle: np.ndarray,
    thetareq: np.ndarray,
    distreq: np.ndarray,
    h: float = 1e-4,
) -> np.ndarray:
    """Finite-difference local gradient from the original notebook.

    As in the original, measured bearings and required distances reconstruct
    the local geometry before differentiating its angular energy.
    """
    n = len(distreq)
    dist = distreq[j]
    v = np.zeros((n, 2))
    v[1:] = np.column_stack((dist * np.cos(angle), dist * np.sin(angle)))
    grad = np.zeros_like(v)

    for i in range(1, n):
        v[i, 0] += h
        e_plus_x = get_energy(v, thetareq[j])
        v[i, 0] -= 2 * h
        e_minus_x = get_energy(v, thetareq[j])
        v[i, 0] += h
        v[i, 1] += h
        e_plus_y = get_energy(v, thetareq[j])
        v[i, 1] -= 2 * h
        e_minus_y = get_energy(v, thetareq[j])
        v[i, 1] += h
        grad[i] = ((e_plus_x - e_minus_x) / (2 * h), (e_plus_y - e_minus_y) / (2 * h))

    grad[0] = -grad[1:].sum(axis=0)
    return grad


def regular_polygon_requirements(n: int, side_length: float = 5.0) -> tuple[np.ndarray, np.ndarray]:
    """Required local angles and distances for a labelled regular ``n``-gon."""
    if n < 3:
        raise ValueError("n must be at least 3.")
    radius = side_length / (2.0 * np.sin(np.pi / n))
    polar = 2.0 * np.pi * np.arange(n) / n
    target = np.column_stack((radius * np.cos(polar), radius * np.sin(polar)))
    thetareq = np.empty((n, n - 2))
    distreq = np.empty((n, n - 1))
    for i in range(n):
        local = np.roll(target, -i, axis=0)
        thetareq[i] = [get_angle(local[0], local[1], local[k]) for k in range(2, n)]
        distreq[i] = np.linalg.norm(local[1:] - local[0], axis=1)
    return thetareq, distreq


def evolve(
    positions: np.ndarray,
    thetareq: np.ndarray,
    distreq: np.ndarray,
    rng: np.random.Generator,
    alpha: float = 1e-3,
    bearing_error_std: float = BEARING_ERROR_STD,
    movement_error_std: float = MOVEMENT_ERROR_STD,
    gradient_threshold: float = 1e-2,
) -> tuple[np.ndarray, float, np.ndarray, np.ndarray]:
    """Run one update; bots below the gradient threshold remain stationary."""
    n = len(positions)
    grads = np.zeros_like(positions)
    energy = 0.0
    ref = np.array([1.0, 0.0])
    for i in range(n):
        local = np.roll(positions, -i, axis=0)
        bearings = np.array([get_angle(local[0], local[0] + ref, point) for point in local[1:]])
        # A 5% Gaussian bearing error is relative to the measured bearing.
        noisy_bearings = bearings * (1.0 + rng.normal(0.0, bearing_error_std, size=n - 1))
        grads += np.roll(get_new_grads(i, noisy_bearings, thetareq, distreq), i, axis=0)
        energy += get_energy(local, thetareq[i])

    gradient_norms = np.linalg.norm(grads, axis=1)
    moving = gradient_norms > gradient_threshold
    executed_motion = np.zeros_like(positions)
    # Error is an additive two-dimensional drift, independent of the
    # gradient direction. Stationary robots receive neither a command nor
    # drift, so the formation remains still once all gradients are small.
    executed_motion[moving] = (
        -alpha * grads[moving]
        + rng.normal(0.0, movement_error_std, size=(moving.sum(), 2))
    )
    return positions + executed_motion, energy, grads, moving


def simulate_polygon(
    n: int,
    seed: int | None = None,
    gradient_threshold: float = 1e-2,
    max_steps: int = 5_000,
    alpha: float = 1e-3,
    initial_spread: float = 2.0,
) -> SimulationResult:
    """Simulate until global energy is less than ``0.05 + 0.025 * n``.

    ``max_steps`` prevents an animation from running indefinitely if a noisy
    trial does not converge; inspect ``result.converged`` in that case.
    """
    if gradient_threshold <= 0:
        raise ValueError("gradient_threshold must be positive.")
    if max_steps < 1:
        raise ValueError("max_steps must be positive.")
    rng = np.random.default_rng(seed)
    thetareq, distreq = regular_polygon_requirements(n)
    current = rng.normal(size=(n, 2)) * initial_spread
    history = [current.copy()]
    energies: list[float] = []
    gradient_history: list[np.ndarray] = []
    energy_threshold = 0.05 + 0.025 * n
    converged = False

    for _ in range(max_steps):
        next_positions, energy, grads, _ = evolve(
            current, thetareq, distreq, rng, alpha=alpha,
            gradient_threshold=gradient_threshold,
        )
        energies.append(energy)
        gradient_history.append(np.linalg.norm(grads, axis=1))
        if energy < energy_threshold:
            converged = True
            break
        current = next_positions
        history.append(current.copy())
    return SimulationResult(
        positions=np.asarray(history),
        energies=np.asarray(energies),
        gradient_norms=np.asarray(gradient_history),
        converged=converged,
        energy_threshold=energy_threshold,
    )


def animate_polygon(
    n: int,
    seed: int | None = None,
    gradient_threshold: float = 1e-2,
    max_steps: int = 5_000,
    alpha: float = 1e-3,
    interval: int = 40,
):
    """Return a Matplotlib animation of the noisy regular-``n``-gon simulation."""
    import matplotlib as mpl
    mpl.rcParams['animation.embed_limit'] = 100.0
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    result = simulate_polygon(n, seed, gradient_threshold, max_steps, alpha)
    frames = result.positions
    extent = max(3.0, float(np.abs(frames).max()) * 1.15)
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal", adjustable="box")
    ax.set(xlim=(-extent, extent), ylim=(-extent, extent), xlabel="x", ylabel="y")
    points = ax.scatter([], [], color="crimson", s=45, zorder=2)
    polygon, = ax.plot([], [], color="steelblue", lw=1.5, zorder=1)
    title = ax.set_title("")

    def draw(frame: int):
        pos = frames[frame]
        points.set_offsets(pos)
        polygon.set_data(np.r_[pos[:, 0], pos[0, 0]], np.r_[pos[:, 1], pos[0, 1]])
        energy = result.energies[min(frame, len(result.energies) - 1)] if len(result.energies) else np.nan
        max_grad = result.gradient_norms[min(frame, len(result.gradient_norms) - 1)].max()
        title.set_text(f"{n}-gon | step {frame} | energy {energy:.4f} | max |grad| {max_grad:.4f}")
        return points, polygon, title

    animation = FuncAnimation(fig, draw, frames=len(frames), interval=interval, blit=True, repeat=False)
    # Keep these useful results accessible without changing the conventional return type.
    animation.simulation_result = result
    return animation


__all__: Iterable[str] = (
    "BEARING_ERROR_STD", "MOVEMENT_ERROR_STD", "SimulationResult", "animate_polygon",
    "evolve", "get_angle", "get_energy", "get_new_grads", "regular_polygon_requirements", "simulate_polygon",
)
