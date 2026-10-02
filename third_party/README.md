# Third-party dependencies

These packages are not vendored. `dependencies.rosinstall` pins the upstream commits used on the vehicle and
`patches/` holds the local modifications as plain `git diff` output.

| Package | Upstream | What the patch changes |
| --- | --- | --- |
| `ublox` | KumarRobotics/ublox | ZED-F9P config (115200 baud, NAV-PVT / RELPOSNED output), node name fixed to `ublox`, NAV-PVT heading republished in degrees for the planner |
| `gps_umd` | swri-robotics/gps_umd | `utm_odometry_node` reads `/ublox/fix` and publishes UTM odometry on `/odom/filtered` |
| `HesaiLidar_General_ROS` | HesaiTechnology/HesaiLidar_General_ROS | PandarXT-16 launch defaults, `log.h` ported from deprecated `ftime()` to `std::chrono`, point type tweaks |
| `morai_msgs` | morai-developergroup/morai_msgs | `CtrlCmd` extended with `seq` and `gear`; adds `LocalControl`, `LocalTrack`, `Estop`, `BoundingBox(es)`, `TrafficCustom` |
| `rtcm_msgs` | tilk/rtcm_msgs | unmodified |
| `usb_cam` | ros-drivers/usb_cam 0.3.6 | unmodified |
