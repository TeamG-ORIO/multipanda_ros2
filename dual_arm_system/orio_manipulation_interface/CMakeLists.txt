cmake_minimum_required(VERSION 3.8)
project(orio_manipulation_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orio_dual_arm_msgs REQUIRED)

# Header-only library so the task supervisor can include the interface
add_library(${PROJECT_NAME}_headers INTERFACE)
target_include_directories(${PROJECT_NAME}_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_headers INTERFACE
  rclcpp geometry_msgs orio_dual_arm_msgs)

# Stand-alone manipulation server node
add_executable(manipulation_server src/manipulation_server.cpp)
target_include_directories(manipulation_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(manipulation_server PUBLIC cxx_std_17)
ament_target_dependencies(manipulation_server
  rclcpp geometry_msgs orio_dual_arm_msgs)

install(TARGETS manipulation_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
