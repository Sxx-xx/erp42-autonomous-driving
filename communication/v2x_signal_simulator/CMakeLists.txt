cmake_minimum_required(VERSION 3.0.2)
project(v2x_signal_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  control_msgs
  morai_msgs
  ublox_msgs
  vehicle_msgs
  v2x_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    control_msgs
    morai_msgs
    ublox_msgs
    vehicle_msgs
    v2x_msgs
)

catkin_install_python(PROGRAMS
  scripts/v2x_signal_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
