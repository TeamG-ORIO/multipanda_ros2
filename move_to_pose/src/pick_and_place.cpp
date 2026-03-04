#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using moveit::planning_interface::MoveGroupInterface;

// Build a box collision object for MoveIt's planning scene.
moveit_msgs::msg::CollisionObject make_box(
  const std::string & id,
  const std::string & frame_id,
  std::array<double, 3> pos_xyz,
  std::array<double, 3> half_extents)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {
    2.0 * half_extents[0],
    2.0 * half_extents[1],
    2.0 * half_extents[2]
  };

  geometry_msgs::msg::Pose pose;
  pose.position.x = pos_xyz[0];
  pose.position.y = pos_xyz[1];
  pose.position.z = pos_xyz[2];
  pose.orientation.w = 1.0;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  return obj;
}

// Convert position + ZYX Euler angles (degrees) to a Pose message.
geometry_msgs::msg::Pose euler_to_pose(
  double x, double y, double z,
  double roll_deg, double pitch_deg, double yaw_deg)
{
  constexpr double DEG2RAD = M_PI / 180.0;
  double roll  = roll_deg  * DEG2RAD;
  double pitch = pitch_deg * DEG2RAD;
  double yaw   = yaw_deg   * DEG2RAD;

  double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);
  double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);

  geometry_msgs::msg::Pose msg;
  msg.orientation.w = cr * cp * cy + sr * sp * sy;
  msg.orientation.x = sr * cp * cy - cr * sp * sy;
  msg.orientation.y = cr * sp * cy + sr * cp * sy;
  msg.orientation.z = cr * cp * sy - sr * sp * cy;
  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;
  return msg;
}

// Plan and execute a Cartesian path through the given waypoints.
// Returns true if the achieved fraction meets the minimum threshold.
bool cart_execute(
  MoveGroupInterface & mgi,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  double eef_step, double jump_threshold, double min_fraction,
  const rclcpp::Logger & logger)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = mgi.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Cartesian path: %.1f%% achieved.", fraction * 100.0);

  if (fraction >= min_fraction) {
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    mgi.execute(plan);
    return true;
  }

  RCLCPP_ERROR(
    logger,
    "Cartesian planning failed: %.1f%% achieved (min: %.1f%%).",
    fraction * 100.0, min_fraction * 100.0);
  return false;
}

// Move the end-effector by dz along the world Z axis from a known pose.
// Uses an explicit start pose instead of querying getCurrentPose() to avoid
// failures when the simulator clock is out of sync with wall time.
bool move_z_from(
  MoveGroupInterface & mgi,
  const geometry_msgs::msg::Pose & from,
  double dz,
  double eef_step, double jump_threshold, double min_fraction,
  const rclcpp::Logger & logger)
{
  geometry_msgs::msg::Pose target = from;
  target.position.z += dz;

  std::vector<geometry_msgs::msg::Pose> waypoints = {target};
  return cart_execute(mgi, waypoints, eef_step, jump_threshold, min_fraction, logger);
}

void activate_gripper(const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Activating gripper...");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(logger, "Gripper activated.");
}

void deactivate_gripper(const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Deactivating gripper...");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(logger, "Gripper deactivated.");
}

// Move to the named "ready" configuration using joint-space planning.
bool move_to_ready(MoveGroupInterface & mgi, const rclcpp::Logger & logger)
{
  mgi.setNamedTarget("ready");
  MoveGroupInterface::Plan plan;
  bool ok = static_cast<bool>(mgi.plan(plan));
  if (ok) {
    mgi.execute(plan);
    return true;
  }
  RCLCPP_ERROR(logger, "Failed to plan to ready position.");
  return false;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pick_and_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("pick_and_place");

  // Declare parameters with defaults in case the YAML is not loaded.
  auto declare_if_not = [&](const std::string & name, double default_val) {
    if (!node->has_parameter(name)) node->declare_parameter(name, default_val);
  };
  declare_if_not("grasp_pose.x",     0.4);
  declare_if_not("grasp_pose.y",     0.0);
  declare_if_not("grasp_pose.z",     0.3);
  declare_if_not("grasp_pose.roll",  180.0);
  declare_if_not("grasp_pose.pitch", 0.0);
  declare_if_not("grasp_pose.yaw",   0.0);
  declare_if_not("place_pose.x",     0.4);
  declare_if_not("place_pose.y",     0.3);
  declare_if_not("place_pose.z",     0.3);
  declare_if_not("place_pose.roll",  180.0);
  declare_if_not("place_pose.pitch", 0.0);
  declare_if_not("place_pose.yaw",   0.0);
  declare_if_not("approach_distance", 0.1);
  declare_if_not("eef_step",          0.01);
  declare_if_not("jump_threshold",    0.0);
  declare_if_not("min_fraction",      0.9);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto mgi = MoveGroupInterface(node, "panda_manipulator");
  const std::string frame = mgi.getPlanningFrame();

  // --- Load parameters ---
  const double approach = node->get_parameter("approach_distance").as_double();
  const double eef_step       = node->get_parameter("eef_step").as_double();
  const double jump_threshold = node->get_parameter("jump_threshold").as_double();
  const double min_fraction   = node->get_parameter("min_fraction").as_double();

  const geometry_msgs::msg::Pose grasp_pose = euler_to_pose(
    node->get_parameter("grasp_pose.x").as_double(),
    node->get_parameter("grasp_pose.y").as_double(),
    node->get_parameter("grasp_pose.z").as_double(),
    node->get_parameter("grasp_pose.roll").as_double(),
    node->get_parameter("grasp_pose.pitch").as_double(),
    node->get_parameter("grasp_pose.yaw").as_double());

  const geometry_msgs::msg::Pose place_pose = euler_to_pose(
    node->get_parameter("place_pose.x").as_double(),
    node->get_parameter("place_pose.y").as_double(),
    node->get_parameter("place_pose.z").as_double(),
    node->get_parameter("place_pose.roll").as_double(),
    node->get_parameter("place_pose.pitch").as_double(),
    node->get_parameter("place_pose.yaw").as_double());

  // --- Populate planning scene ---
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(make_box("table", frame, {0.0, 0.0, -0.5}, {0.4, 0.6, 0.5}));
  collision_objects.push_back(make_box("left_post",  frame, {0.0, -0.597, 0.5}, {0.02, 0.02, 0.5}));
  collision_objects.push_back(make_box("right_post", frame, {0.0,  0.597, 0.5}, {0.02, 0.02, 0.5}));
  collision_objects.push_back(make_box("crossbar",   frame, {0.0, 0.0, 1.02},   {0.02, 0.6,  0.02}));

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(collision_objects);
  RCLCPP_INFO(logger, "Planning scene populated.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  {
    auto known = psi.getKnownObjectNames();
    RCLCPP_INFO(logger, "Scene objects confirmed: %zu", known.size());
    if (known.size() != collision_objects.size()) {
      RCLCPP_WARN(logger, "Scene object count mismatch — waiting another 500 ms");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  // =========================================================================
  // PICK sequence
  // =========================================================================

  // 1. Move to approach pose above grasp (grasp_pose with z += approach)
  RCLCPP_INFO(logger, "[PICK] Moving to approach pose above grasp.");
  {
    geometry_msgs::msg::Pose approach_pose = grasp_pose;
    approach_pose.position.z += approach;
    std::vector<geometry_msgs::msg::Pose> waypoints = {approach_pose};
    cart_execute(mgi, waypoints, eef_step, jump_threshold, min_fraction, logger);
  }

  // 2. Descend to grasp pose
  RCLCPP_INFO(logger, "[PICK] Descending to grasp pose.");
  {
    geometry_msgs::msg::Pose approach_pose = grasp_pose;
    approach_pose.position.z += approach;
    move_z_from(mgi, approach_pose, -approach, eef_step, jump_threshold, min_fraction, logger);
  }

  // 3. Close gripper
  activate_gripper(logger);

  // 4. Retract upward
  RCLCPP_INFO(logger, "[PICK] Retracting.");
  move_z_from(mgi, grasp_pose, approach, eef_step, jump_threshold, min_fraction, logger);

  // 5. Move to ready
  RCLCPP_INFO(logger, "[PICK] Moving to ready position.");
  move_to_ready(mgi, logger);

  // =========================================================================
  // PLACE sequence
  // =========================================================================

  // 6. Move to approach pose above place (place_pose with z += approach)
  RCLCPP_INFO(logger, "[PLACE] Moving to approach pose above place.");
  {
    geometry_msgs::msg::Pose approach_pose = place_pose;
    approach_pose.position.z += approach;
    std::vector<geometry_msgs::msg::Pose> waypoints = {approach_pose};
    cart_execute(mgi, waypoints, eef_step, jump_threshold, min_fraction, logger);
  }

  // 7. Descend to place pose
  RCLCPP_INFO(logger, "[PLACE] Descending to place pose.");
  {
    geometry_msgs::msg::Pose approach_pose = place_pose;
    approach_pose.position.z += approach;
    move_z_from(mgi, approach_pose, -approach, eef_step, jump_threshold, min_fraction, logger);
  }

  // 8. Open gripper
  deactivate_gripper(logger);

  // 9. Retract upward
  RCLCPP_INFO(logger, "[PLACE] Retracting.");
  move_z_from(mgi, place_pose, approach, eef_step, jump_threshold, min_fraction, logger);

  // 10. Move to ready
  RCLCPP_INFO(logger, "[PLACE] Moving to ready position.");
  move_to_ready(mgi, logger);

  RCLCPP_INFO(logger, "Pick and place complete.");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
