cmake_minimum_required(VERSION 3.8)
project(orio_dual_arm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExecutionResult.msg"
  "msg/GraspPose.msg"
  "msg/LabelPose.msg"
  "msg/SystemStatus.msg"
  "srv/ArmCommand.srv"
  "srv/MoveToPose.srv"
  "srv/GripperCommand.srv"
  "srv/GetGraspPose.srv"
  "srv/GetLabelPose.srv"
  "srv/SupervisorCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
