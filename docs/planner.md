# Planner design

## Decision

`RevisitSelector` picks the old keyframe whose recognition would remove the
most pose uncertainty per metre driven.

1. **Pose graph.** Nodes, odometry links and closures come from `mapData`.
   Every link is an information matrix: odometry drift per metre and per radian
   turned, closure noise for closures.
2. **Uncertainty.** With the robot pose fixed, each keyframe has a relative
   covariance Σ. A keyframe is only worth a revisit when its position standard
   deviation exceeds `max_position_std_m`, the free room beside the robot.
3. **Gain.** `gain = ½ ln(det Σ / det Σ′)`, where Σ′ is the covariance after
   closing there and driving straight back. It has no unit. `gain ≤ 0` is
   skipped, so being far away is no reason by itself.
4. **Score.** `score = P_lc × gain / cost`. The cost is the A* distance plus
   `turning_radius_m` times the angle to turn.
5. **Excluded.** Targets within `arrival_xy_m`, because Nav2 would report
   arrival without moving, and targets visited before without a closure.

`P_lc` estimates how recognisable a keyframe is: the number of its visual words
(local saliency) and their rarity in the map (global saliency), scaled by the
observed closure rate.

## Node

- Decides only between exploration routes: on `route_completed`, or when the
  first goal of a route has just been sent. Without exploration events it
  decides on every map update.
- `alc_started` / `alc_finished` on `planner_event_topic` tell the explorer to
  pause and resume. The Nav2 goal is sent once Nav2 has been idle for 0.5 s.
- The goal faces the way the keyframe did. `nav_behavior_tree` selects a tree
  that arrives along that heading; `spin_at_target` adds a full turn for robots
  that can turn on the spot.
- A goal is cancelled after `navigation_timeout_sec`.
- `alc_evaluated` is published for every decision, whether or not it leads to a
  revisit.

## Limits

- Drift and closure parameters were fitted to one robot. Refit them for another.
- `P_lc` is close to 1 for most keyframes and hardly separates candidates.
- A target that produced no closure is never tried again in the same run.
