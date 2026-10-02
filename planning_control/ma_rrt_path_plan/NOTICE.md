# Modified fork of ma_rrt_path_plan

Upstream: <https://github.com/egnitionHamburg/ma_rrt_path_plan> (via <https://github.com/ekampourakis/ma_rrt_path_plan>), MIT — see `LICENSE`.
`README.md` in this directory is the upstream README.

Changes made for the ERP42 platform:

| File | Purpose |
| --- | --- |
| `src/MaRRTPathPlanNode_oa.py` | Obstacle-avoidance node: subscribes LiDAR clusters on `/track`, groups them into obstacles, picks the gap between the two nearest obstacles as the RRT goal, publishes `/newwaypoints` and `/rrt_newwaypoints` plus RViz goal marker |
| `src/ma_rrt_oa.py` | RRT variant: minimum sampling distance, 30° expansion cone, headless matplotlib |
| `src/main_oa.py`, `src/track_oa.py` | Entry point / dead-reckoning helper for the OA pipeline |
| `launch/startExploring_oa.launch`, `launch/lidar_all.launch` | OA launch, and all-in-one LiDAR + clustering + RRT launch |
| `rviz/` | RViz configs |

`vehicle_msgs` was moved out to `msgs/vehicle_msgs` and extended (`ObjectInfo`, `AvoidAngleArray`, `Car`, `CarObject`).
