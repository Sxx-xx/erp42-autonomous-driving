cmake_minimum_required(VERSION 3.0.2)
project(v2x_msgs)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Id.msg               # 의존 메시지 먼저 포함
  Interchange.msg
  Spat.msg
  Spat_new.msg         # 만약 이름 변경했다면 이걸로
  IntersectionState.msg
  MovementState.msg
  MovementEvent.msg
  TimeChangeDetails.msg
  State.msg
  State_time_speed.msg
  Timing.msg
  Point2D.msg
)

add_service_files(
  FILES
  RemainingDistance.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs               # 여기에 의존 패키지 명시
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
