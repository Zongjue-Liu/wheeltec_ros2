# Wheeltec Clean Navigation

## Dimensioned course map (2026-07-12)

The active `maps/WHEELTEC.yaml` is generated from the physical training course:

- Physical course: 3.23 m x 3.55 m
- Map resolution: 0.01 m/cell
- White: traversable black road surface on the physical mat
- Black: outer fence and four non-drivable islands
- Gray: outside the measured course, unavailable to planning
- Suggested start: lower-left road, facing map `+Y` (up in RViz)

`course_markers` publishes visual-only crosswalks, direction arrows, and a
green suggested-start arrow on `/course_markers`. These markers do not alter
the costmap. Set `2D Pose Estimate` manually and verify LaserScan overlap before
sending a goal.

Regenerate the installed map after changing measured dimensions:

```bash
ros2 run wheeltec_clean_nav generate_course_map
```

Generate into an explicit source or staging directory:

```bash
ros2 run wheeltec_clean_nav generate_course_map --output-dir /path/to/maps
```

Independent Nav2 and AMCL integration for the Wheeltec mini Ackermann chassis.

Safety defaults:

- AMCL owns `map -> odom_combined`.
- Nav2 publishes `/nav2/cmd_vel` only.
- Traffic-light filter is the intended publisher to chassis `/cmd_vel`.
- Route execution is disabled by default.
- Maximum filtered linear speed is `0.12 m/s`.

Validation launch (no automatic goal):

```bash
ros2 launch wheeltec_clean_nav clean_navigation.launch.py start_route:=false
```

Only after map, scan, AMCL pose, TF, lifecycle nodes and velocity endpoints are
validated should `start_route:=true` be used.
