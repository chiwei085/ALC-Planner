# alc_planner

A ROS2 Humble C++ planner that treats **active loop closure as an intrinsic component of exploration quality**, not a post-hoc SLAM repair action.

During autonomous exploration, SLAM pose drift accumulates and can distort occupancy maps (e.g., sealing off narrow corridors). This planner continuously evaluates the reward of revisiting previously seen locations to trigger loop closures at the right moment, keeping the pose graph stable throughout exploration rather than only at the end.

## How it works

Between exploration routes the planner computes, from the whole pose graph, how
uncertain the robot is relative to every old keyframe. It revisits the keyframe
that removes the most uncertainty per metre driven, weighted by how likely it is
to be recognised. See [docs/planner.md](docs/planner.md).

The problem framing follows Yin et al. (active loop closure as part of
exploration); the recognisability estimate follows the local and global visual
saliency of Kim & Eustice.

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

Subscribes to `map_data_topic` (`/rtabmap/mapData`), `info_topic`
(`/rtabmap/info`) and `map_topic` (`/map`), and sends goals to Nav2's
`navigate_to_pose`. All parameters and their defaults are in
[types.hpp](include/alc_planner/types.hpp) and `planner_node.cpp`.

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
