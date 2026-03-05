#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

// #region agent log
static void dbg_log(const char* loc, const char* msg, const char* arm_name, int line,
  double num_val = -999.0, bool bool_val = false, bool has_num = false, bool has_bool = false) {
  std::ofstream f("/home/developer/multipanda_ws/src/multipanda_ros2/.cursor/debug-b9bb70.log", std::ios::app);
  if (!f) return;
  auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  f << "{\"sessionId\":\"b9bb70\",\"hypothesisId\":\"H1\",\"location\":\"" << loc << ":" << line
    << "\",\"message\":\"" << msg << "\",\"data\":{\"arm\":\"" << arm_name << "\"";
  if (has_num) f << ",\"value\":" << num_val;
  if (has_bool) f << ",\"ok\":" << (bool_val ? "true" : "false");
  f << "},\"timestamp\":" << ts << "}\n";
  f.close();
}
// #endregion

// Build a target pose from ZYX Euler angles (degrees) and a position (metres).
static geometry_msgs::msg::Pose make_pose(
  double x, double y, double z,
  double roll_deg, double pitch_deg, double yaw_deg)
{
  constexpr double DEG2RAD = M_PI / 180.0;
  const double roll  = roll_deg  * DEG2RAD;
  const double pitch = pitch_deg * DEG2RAD;
  const double yaw   = yaw_deg   * DEG2RAD;

  // ZYX Euler (yaw * pitch * roll) -> quaternion
  const double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = cr * cp * cy + sr * sp * sy;
  pose.orientation.x = sr * cp * cy - cr * sp * sy;
  pose.orientation.y = cr * sp * cy + sr * cp * sy;
  pose.orientation.z = cr * cp * sy - sr * sp * cy;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  return pose;
}

// Plan and execute a Cartesian path for one arm. Returns true on success.
static bool cart_move(
  moveit::planning_interface::MoveGroupInterface & mgi,
  const geometry_msgs::msg::Pose & target,
  double eef_step,
  double jump_threshold,
  double min_fraction,
  const rclcpp::Logger & logger,
  const std::string & arm_name)
{
  // #region agent log
  dbg_log("orio_dual_cart_move_to_pose.cpp", "cart_move_enter", arm_name.c_str(), __LINE__);
  // #endregion
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target);

  moveit_msgs::msg::RobotTrajectory trajectory;
  // #region agent log
  dbg_log("orio_dual_cart_move_to_pose.cpp", "before_computeCartesianPath", arm_name.c_str(), __LINE__);
  // #endregion
  const double fraction = mgi.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);
  // #region agent log
  dbg_log("orio_dual_cart_move_to_pose.cpp", "after_computeCartesianPath", arm_name.c_str(), __LINE__, fraction, false, true, false);
  // #endregion

  RCLCPP_INFO(logger, "[%s] Cartesian path: %.1f%% achieved.", arm_name.c_str(), fraction * 100.0);

  if (fraction < min_fraction) {
    RCLCPP_ERROR(logger,
      "[%s] Cartesian planning failed: %.1f%% < required %.1f%%.",
      arm_name.c_str(), fraction * 100.0, min_fraction * 100.0);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  // #region agent log
  dbg_log("orio_dual_cart_move_to_pose.cpp", "before_execute", arm_name.c_str(), __LINE__);
  // #endregion
  const auto result = mgi.execute(plan);
  // #region agent log
  dbg_log("orio_dual_cart_move_to_pose.cpp", "after_execute", arm_name.c_str(), __LINE__, 0.0, false, true, false);
  // #endregion
  const bool ok = (result == moveit::core::MoveItErrorCode::SUCCESS);

  if (!ok) {
    RCLCPP_ERROR(logger, "[%s] Execution failed!", arm_name.c_str());
  } else {
    RCLCPP_INFO(logger, "[%s] Execution succeeded.", arm_name.c_str());
  }
  // #region agent log
  dbg_log("orio_dual_cart_move_to_pose.cpp", "cart_move_exit", arm_name.c_str(), __LINE__, 0, ok, false, true);
  // #endregion
  return ok;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "orio_dual_cart_move_to_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("orio_dual_cart_move_to_pose");

  auto declare_if_not = [&](const std::string & name, double default_val) {
    if (!node->has_parameter(name)) node->declare_parameter(name, default_val);
  };

  // Per-arm target pose parameters
  declare_if_not("left_x",     0.3);
  declare_if_not("left_y",     0.5);
  declare_if_not("left_z",     1.2);
  declare_if_not("left_roll",  180.0);
  declare_if_not("left_pitch", 0.0);
  declare_if_not("left_yaw",   0.0);

  declare_if_not("right_x",     0.3);
  declare_if_not("right_y",     0.1);
  declare_if_not("right_z",     1.2);
  declare_if_not("right_roll",  180.0);
  declare_if_not("right_pitch", 0.0);
  declare_if_not("right_yaw",   0.0);

  // Shared Cartesian planning parameters
  declare_if_not("eef_step",       0.01);
  declare_if_not("jump_threshold", 0.0);
  declare_if_not("min_fraction",   0.9);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto mgi_left  = MoveGroupInterface(node, "mj_left_manipulator");
  auto mgi_right = MoveGroupInterface(node, "mj_right_manipulator");

  // Wait for a valid current state before planning.
  mgi_left.startStateMonitor();
  mgi_left.getCurrentState(2.0);
  mgi_right.startStateMonitor();
  mgi_right.getCurrentState(2.0);

  auto left_pose  = mgi_left.getCurrentPose();
  auto right_pose = mgi_right.getCurrentPose();

  RCLCPP_INFO(logger, "Left  EEF: pos=(%.3f, %.3f, %.3f)",
    left_pose.pose.position.x, left_pose.pose.position.y, left_pose.pose.position.z);
  RCLCPP_INFO(logger, "Right EEF: pos=(%.3f, %.3f, %.3f)",
    right_pose.pose.position.x, right_pose.pose.position.y, right_pose.pose.position.z);

  const double eef_step       = node->get_parameter("eef_step").as_double();
  const double jump_threshold = node->get_parameter("jump_threshold").as_double();
  const double min_fraction   = node->get_parameter("min_fraction").as_double();

  const auto target_left = make_pose(
    node->get_parameter("left_x").as_double(),
    node->get_parameter("left_y").as_double(),
    node->get_parameter("left_z").as_double(),
    node->get_parameter("left_roll").as_double(),
    node->get_parameter("left_pitch").as_double(),
    node->get_parameter("left_yaw").as_double());

  const auto target_right = make_pose(
    node->get_parameter("right_x").as_double(),
    node->get_parameter("right_y").as_double(),
    node->get_parameter("right_z").as_double(),
    node->get_parameter("right_roll").as_double(),
    node->get_parameter("right_pitch").as_double(),
    node->get_parameter("right_yaw").as_double());

  RCLCPP_INFO(logger, "Left  target: pos=(%.3f, %.3f, %.3f)", target_left.position.x,  target_left.position.y,  target_left.position.z);
  RCLCPP_INFO(logger, "Right target: pos=(%.3f, %.3f, %.3f)", target_right.position.x, target_right.position.y, target_right.position.z);

  // Apply left_* pose to mj_left_manipulator and right_* to mj_right_manipulator.
  // #region agent log
  dbg_log("orio_dual_cart_move_to_pose.cpp", "main_before_first_cart_move", "left", __LINE__);
  // #endregion
  cart_move(mgi_left,  target_left,  eef_step, jump_threshold, min_fraction, logger, "left");
  cart_move(mgi_right, target_right, eef_step, jump_threshold, min_fraction, logger, "right");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
