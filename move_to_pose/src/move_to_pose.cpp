#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  auto declare_if_not = [&](const std::string & name, double default_val) {
    if (!node->has_parameter(name)) node->declare_parameter(name, default_val);
  };
  declare_if_not("x",   0.3);
  declare_if_not("y",   0.4);
  declare_if_not("z",   0.4);
  // Rotation matrix rows: r<row><col>
  // Default: 180° around X (end-effector pointing down)
  declare_if_not("r00",  1.0); declare_if_not("r01",  0.0); declare_if_not("r02",  0.0);
  declare_if_not("r10",  0.0); declare_if_not("r11", -1.0); declare_if_not("r12",  0.0);
  declare_if_not("r20",  0.0); declare_if_not("r21",  0.0); declare_if_not("r22", -1.0);

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose
  auto const target_pose = [&]{
    geometry_msgs::msg::Pose msg;

    // Read rotation matrix elements
    double R[3][3];
    R[0][0] = node->get_parameter("r00").as_double();
    R[0][1] = node->get_parameter("r01").as_double();
    R[0][2] = node->get_parameter("r02").as_double();
    R[1][0] = node->get_parameter("r10").as_double();
    R[1][1] = node->get_parameter("r11").as_double();
    R[1][2] = node->get_parameter("r12").as_double();
    R[2][0] = node->get_parameter("r20").as_double();
    R[2][1] = node->get_parameter("r21").as_double();
    R[2][2] = node->get_parameter("r22").as_double();

    // Shepperd's method: rotation matrix -> quaternion
    double trace = R[0][0] + R[1][1] + R[2][2];
    double qx, qy, qz, qw;
    if (trace > 0.0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      qw = 0.25 / s;
      qx = (R[2][1] - R[1][2]) * s;
      qy = (R[0][2] - R[2][0]) * s;
      qz = (R[1][0] - R[0][1]) * s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
      qw = (R[2][1] - R[1][2]) / s;
      qx = 0.25 * s;
      qy = (R[0][1] + R[1][0]) / s;
      qz = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
      double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
      qw = (R[0][2] - R[2][0]) / s;
      qx = (R[0][1] + R[1][0]) / s;
      qy = 0.25 * s;
      qz = (R[1][2] + R[2][1]) / s;
    } else {
      double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
      qw = (R[1][0] - R[0][1]) / s;
      qx = (R[0][2] + R[2][0]) / s;
      qy = (R[1][2] + R[2][1]) / s;
      qz = 0.25 * s;
    }

    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.position.x = node->get_parameter("x").as_double();
    msg.position.y = node->get_parameter("y").as_double();
    msg.position.z = node->get_parameter("z").as_double();
    return msg;
  }();

  move_group_interface.setPoseTarget(target_pose);

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
  return 0;
}