# alc_planner

A ROS2 Humble C++ planner that treats **active loop closure as an intrinsic component of exploration quality**, not a post-hoc SLAM repair action.

During autonomous exploration, SLAM pose drift accumulates and can distort occupancy maps (e.g., sealing off narrow corridors). This planner continuously evaluates the reward of revisiting previously seen locations to trigger loop closures at the right moment, keeping the pose graph stable throughout exploration rather than only at the end.

## Design basis

The planner synthesizes two papers:

- **Ref1 (Yin et al., 2024)** — probabilistic reward framework: travel cost vs. uncertainty-reduction-weighted loop closure probability; distance-based uncertainty surrogate; Branch-and-Bound target selection; adaptive triggering threshold.
- **Ref2 (Kim & Eustice, 2013)** — perception-driven decision formulation: dual saliency metrics (local texture richness + global scene rarity); DBSCAN-based waypoint clustering; saliency-weighted A*; area coverage term in reward.

What we deliberately do not adopt verbatim: Ref1's cluster P_LC independence assumption (nearby keyframe loop closure events are correlated); Ref2's empirical P_L model (task-specific prior, not transferable to indoor frontier exploration); and Ref2's linear uncertainty–coverage weighting (the exchangeability of these two quantities is not justified).

See [docs/planner.md](docs/planner.md) for the full implementation plan.

## Environment

- ROS2 Humble
- RTAbMap (SLAM backend)
- Nav2 (navigation)
- C++17

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- `rosdep`

Recommended installation:

```bash
source /opt/ros/humble/setup.bash
cd <your_ws>
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## Installation

```bash
cd <your_ws>
colcon build --packages-select alc_planner
source install/setup.bash
```

## Run

```bash
ros2 launch alc_planner alc_planner.launch.py
```

## References

```bibtex
@article{Yin2024,
  author  = {He Yin and Jong Jin Park and Marcelino Almeida and
             Martin Labrie and Jim Zamiska and Richard Kim},
  title   = {Probabilistic Active Loop Closure for Autonomous Exploration},
  year    = {2024},
  url     = {https://www.amazon.science/publications/probabilistic-active-loop-closure-for-autonomous-exploration},
}

@article{Kim2013,
  author  = {Ayoung Kim and Ryan M. Eustice},
  title   = {Active Visual SLAM for Robotic Area Coverage: Theory and Experiment},
  journal = {The International Journal of Robotics Research},
  year    = {2015},
}
```
