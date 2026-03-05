cmake_minimum_required(VERSION 3.8)
project(move_to_pose)

# So clangd (and other tools) get include paths from compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(move_to_pose src/move_to_pose.cpp)
target_include_directories(move_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_pose
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "shape_msgs"
  "geometry_msgs"
  "rclcpp"
)

add_executable(dual_move_to_pose src/dual_move_to_pose.cpp)
target_include_directories(dual_move_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dual_move_to_pose PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  dual_move_to_pose
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "shape_msgs"
  "geometry_msgs"
  "rclcpp"
)

add_executable(cart_move_to_pose src/cart_move_to_pose.cpp)
target_include_directories(cart_move_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cart_move_to_pose PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  cart_move_to_pose
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "shape_msgs"
  "geometry_msgs"
  "rclcpp"
)

add_executable(orio_dual_cart_move_to_pose src/orio_dual_cart_move_to_pose.cpp)
target_include_directories(orio_dual_cart_move_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(orio_dual_cart_move_to_pose PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  orio_dual_cart_move_to_pose
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "shape_msgs"
  "geometry_msgs"
  "rclcpp"
)

add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_and_place PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pick_and_place
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "shape_msgs"
  "geometry_msgs"
  "rclcpp"
)

install(TARGETS move_to_pose dual_move_to_pose cart_move_to_pose orio_dual_cart_move_to_pose pick_and_place
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
