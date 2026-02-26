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
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// Build a box collision object for MoveIt's planning scene.
// pos_xyz: center position (metres), half_extents: box half-sizes (metres).
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
  // MoveIt BOX dimensions are full extents, MuJoCo size is half-extents
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

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "move_to_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_to_pose");

  auto declare_if_not = [&](const std::string & name, double default_val) {
    if (!node->has_parameter(name)) node->declare_parameter(name, default_val);
  };
  declare_if_not("x",     0.3);
  declare_if_not("y",     0.4);
  declare_if_not("z",     0.4);
  declare_if_not("roll",  180.0);
  declare_if_not("pitch", 0.0);
  declare_if_not("yaw",   0.0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "dual_panda");

  // Set a target Pose
  // auto const target_pose = [&]{
  //   geometry_msgs::msg::Pose msg;

  //   constexpr double DEG2RAD = M_PI / 180.0;
  //   double roll  = node->get_parameter("roll").as_double()  * DEG2RAD;
  //   double pitch = node->get_parameter("pitch").as_double() * DEG2RAD;
  //   double yaw   = node->get_parameter("yaw").as_double()   * DEG2RAD;

  //   // ZYX Euler (yaw * pitch * roll) -> quaternion
  //   double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);
  //   double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  //   double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);

  //   msg.orientation.w = cr * cp * cy + sr * sp * sy;
  //   msg.orientation.x = sr * cp * cy - cr * sp * sy;
  //   msg.orientation.y = cr * sp * cy + sr * cp * sy;
  //   msg.orientation.z = cr * cp * sy - sr * sp * cy;
  //   msg.position.x = node->get_parameter("x").as_double();
  //   msg.position.y = node->get_parameter("y").as_double();
  //   msg.position.z = node->get_parameter("z").as_double();
  //   return msg;
  // }();

  auto left_pose  = move_group_interface.getCurrentPose("mj_left_link8");
  auto right_pose = move_group_interface.getCurrentPose("mj_right_link8");

  RCLCPP_INFO(logger, "Left  EEF pose: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
    left_pose.pose.position.x,
    left_pose.pose.position.y,
    left_pose.pose.position.z,
    left_pose.pose.orientation.x,
    left_pose.pose.orientation.y,
    left_pose.pose.orientation.z,
    left_pose.pose.orientation.w);

  RCLCPP_INFO(logger, "Right EEF pose: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
    right_pose.pose.position.x,
    right_pose.pose.position.y,
    right_pose.pose.position.z,
    right_pose.pose.orientation.x,
    right_pose.pose.orientation.y,
    right_pose.pose.orientation.z,
    right_pose.pose.orientation.w);

  // Use current orientation so IK only needs to solve for position change
  auto const target_pose_left = [&]{
    geometry_msgs::msg::Pose msg;
    msg.orientation = left_pose.pose.orientation;
    msg.position.x = 0.3;
    msg.position.y = 0.26;
    msg.position.z = 0.6;
    return msg;
  }();

  auto const target_pose_right = [&]{
    geometry_msgs::msg::Pose msg;
    msg.orientation = right_pose.pose.orientation;
    msg.position.x = 0.3;
    msg.position.y = -0.26;
    msg.position.z = 0.6;
    return msg;
  }();

  bool left_set  = move_group_interface.setPoseTarget(target_pose_left,  "mj_left_link8");
  bool right_set = move_group_interface.setPoseTarget(target_pose_right, "mj_right_link8");
  RCLCPP_INFO(logger, "setPoseTarget: left=%s right=%s",
    left_set ? "OK" : "FAILED", right_set ? "OK" : "FAILED");

  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effectors: %s",  move_group_interface.getEndEffectorLink().c_str());

  RCLCPP_INFO(logger, "Goal joint tolerance: %.4f", move_group_interface.getGoalJointTolerance());

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}