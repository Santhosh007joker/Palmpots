# Swarm Robotics DDQN

A six-robot simulator and training implementation for the formation-control proposal in `RL_Solution.md`.

## Model

Each labelled robot observes five body-frame bearings and the global headings of all six robots. For each of the other five robots, it recommends one of thirteen primitives: no movement, one of six 60°-spaced body-frame vectors with 0.5 cm magnitude, or the same six vectors with 1 cm magnitude. The recipient sums incoming body-frame recommendations and projects that sum onto its closest executable primitive.

The learning policy is one shared Double DQN. Sharing its weights keeps the robots identical while every `(sender, recipient)` recommendation still has its own transition and reward. Reward is the sender's marginal local-energy improvement: its local energy after all recommendations are aggregated and executed is compared with the result of executing the same aggregate with that one recommendation removed. It deliberately contains no global-energy term.

The target is a labelled regular hexagon by default. It can be changed in `formation_env.py` by replacing `target_positions` with any six labelled points.

## Install and run

Use Python 3.10+ and install the requirements:

```powershell
python -m pip install -r requirements.txt
python train.py --episodes 3000 --checkpoint checkpoints/hexagon_ddqn.pt
python simulate.py --checkpoint checkpoints/hexagon_ddqn.pt --seed 42
```

To train six independent DQNs instead—one fixed policy per labelled sender—run:

```powershell
python train_six_dqns.py --episodes 3000 --checkpoint checkpoints/six_independent_ddqns.pt
```

It prints per-bot reward, local energy, and TD loss every 25 episodes, and writes the full episode history to `metrics/six_independent_ddqns.csv`.

To save the visual rollout:

```powershell
python simulate.py --checkpoint checkpoints/hexagon_ddqn.pt --seed 42 --save outputs/rollout.gif
```

`simulate.py` starts all six bots from a randomized collision-free configuration and displays the target labels, current robot locations, and global formation energy.

## Important limitation

The proposed angular energy deliberately describes a formation up to scale: it learns the shape but does not enforce a particular inter-robot distance. The simulator does not use distance estimates as state inputs or in the reward. It does retain a hard collision terminal condition as a simulation safety check, not as an energy term.
