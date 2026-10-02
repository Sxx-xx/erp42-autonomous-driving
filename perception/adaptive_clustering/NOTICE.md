# Modified fork of yzrobot/adaptive_clustering

Upstream: <https://github.com/yzrobot/adaptive_clustering> (commit `3e30f31`), BSD 3-Clause — see `LICENSE`.
`README.md` in this directory is the upstream README.

Changes made for the ERP42 platform:

| File | Purpose |
| --- | --- |
| `src/adaptive_clustering.cpp` | ROI (x/y/z + azimuth) filtering, 3D bounding boxes, nearest-cluster markers, `vehicle_msgs/Track` output for the track mission |
| `src/adaptive_clustering_oa.cpp` | Obstacle-avoidance variant: publishes obstacle centroids on `/track` for the RRT planner |
| `src/adaptive_clustering_oa_with_solo.cpp` | OA variant + `/adaptive_clustering/box_drawn` (Bool) used by the mission planner as an "obstacle present" flag; ROI configurable from launch args |
| `src/adaptive_clustering_solo.cpp` | Detection-only variant publishing `is_obstacle` (Bool) |
| `launch/adaptive_clustering_oa.launch` | Launch for the OA pipeline (Hesai PandarXT-16) |
