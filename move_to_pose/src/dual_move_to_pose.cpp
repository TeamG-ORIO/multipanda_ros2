#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>

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

  // Create per-arm MoveGroup interfaces. The combined "dual_panda" group-of-groups
  // has no IK solver itself; setPoseTarget must be called on the individual subgroups
  // (mj_left_manipulator / mj_right_manipulator) which have KDL solvers configured.
  using moveit::planning_interface::MoveGroupInterface;
  auto mgi_left  = MoveGroupInterface(node, "mj_left_manipulator");
  auto mgi_right = MoveGroupInterface(node, "mj_right_manipulator");
  auto mgi_dual  = MoveGroupInterface(node, "dual_panda");

  // Wait for a valid current state before querying poses or planning.
  mgi_left.startStateMonitor();
  mgi_left.getCurrentState(2.0);

  auto left_pose  = mgi_left.getCurrentPose("mj_left_link8");
  auto right_pose = mgi_right.getCurrentPose("mj_right_link8");

  RCLCPP_INFO(logger, "Left  EEF pose: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
    left_pose.pose.position.x,  left_pose.pose.position.y,  left_pose.pose.position.z,
    left_pose.pose.orientation.x, left_pose.pose.orientation.y,
    left_pose.pose.orientation.z, left_pose.pose.orientation.w);

  RCLCPP_INFO(logger, "Right EEF pose: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
    right_pose.pose.position.x,  right_pose.pose.position.y,  right_pose.pose.position.z,
    right_pose.pose.orientation.x, right_pose.pose.orientation.y,
    right_pose.pose.orientation.z, right_pose.pose.orientation.w);

  // Use current orientation so IK only needs to solve for position change
  auto const target_pose_left = [&]{
    geometry_msgs::msg::Pose msg;
    msg.orientation = left_pose.pose.orientation;
    msg.position.x = 0.3;
    msg.position.y = 0.26;
    msg.position.z = 0.4;
    return msg;
  }();

  auto const target_pose_right = [&]{
    geometry_msgs::msg::Pose msg;
    msg.orientation = right_pose.pose.orientation;
    msg.position.x = 0.3;
    msg.position.y = -0.26;
    msg.position.z = 0.4;
    return msg;
  }();

  // Set pose targets on the individual arm groups (each has a KDL IK solver)
  bool left_set  = mgi_left.setPoseTarget(target_pose_left,   "mj_left_link8");
  bool right_set = mgi_right.setPoseTarget(target_pose_right, "mj_right_link8");
  RCLCPP_INFO(logger, "setPoseTarget: left=%s right=%s",
    left_set ? "OK" : "FAILED", right_set ? "OK" : "FAILED");


  RCLCPP_INFO(logger, "Planning frame: %s", mgi_dual.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Goal joint tolerance: %.4f", mgi_dual.getGoalJointTolerance());

  // Plan each arm independently
  MoveGroupInterface::Plan left_plan, right_plan;
  bool left_ok  = static_cast<bool>(mgi_left.plan(left_plan));
  bool right_ok = static_cast<bool>(mgi_right.plan(right_plan));


  RCLCPP_INFO(logger, "Per-arm planning: left=%s right=%s",
    left_ok ? "OK" : "FAILED", right_ok ? "OK" : "FAILED");

  if (left_ok && right_ok) {
    // Merge both single-arm trajectories into one combined plan covering all 14 joints.
    // The dual_panda_arm_controller requires all joints in a single trajectory message.
    MoveGroupInterface::Plan combined_plan;
    auto & lt = left_plan.trajectory_.joint_trajectory;
    auto & rt = right_plan.trajectory_.joint_trajectory;

    combined_plan.trajectory_.joint_trajectory.header = lt.header;

    // Joint names: left arm first, then right
    combined_plan.trajectory_.joint_trajectory.joint_names = lt.joint_names;
    for (const auto & n : rt.joint_names) {
      combined_plan.trajectory_.joint_trajectory.joint_names.push_back(n);
    }

    // Align trajectories by time: use the longer trajectory's time stamps,
    // padding the shorter one by repeating its final point.
    const auto & l_traj = left_plan.trajectory_.joint_trajectory;
    const auto & r_traj = right_plan.trajectory_.joint_trajectory;
    size_t n_pts = std::max(l_traj.points.size(), r_traj.points.size());

    for (size_t i = 0; i < n_pts; ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      const auto & lp = l_traj.points[std::min(i, l_traj.points.size() - 1)];
      const auto & rp = r_traj.points[std::min(i, r_traj.points.size() - 1)];

      pt.positions.insert(pt.positions.end(), lp.positions.begin(), lp.positions.end());
      pt.positions.insert(pt.positions.end(), rp.positions.begin(), rp.positions.end());

      // Handle velocities if present
      if (!lp.velocities.empty())
        pt.velocities.insert(pt.velocities.end(), lp.velocities.begin(), lp.velocities.end());
      else
        pt.velocities.resize(pt.velocities.size() + lp.positions.size(), 0.0);
        
      if (!rp.velocities.empty())
        pt.velocities.insert(pt.velocities.end(), rp.velocities.begin(), rp.velocities.end());
      else
        pt.velocities.resize(pt.velocities.size() + rp.positions.size(), 0.0);

      // Handle accelerations if present
      if (!lp.accelerations.empty())
        pt.accelerations.insert(pt.accelerations.end(), lp.accelerations.begin(), lp.accelerations.end());
      else
        pt.accelerations.resize(pt.accelerations.size() + lp.positions.size(), 0.0);

      if (!rp.accelerations.empty())
        pt.accelerations.insert(pt.accelerations.end(), rp.accelerations.begin(), rp.accelerations.end());
      else
        pt.accelerations.resize(pt.accelerations.size() + rp.positions.size(), 0.0);

      // Use the later time_from_start of the two points
      auto l_time = rclcpp::Duration(lp.time_from_start).seconds();
      auto r_time = rclcpp::Duration(rp.time_from_start).seconds();
      pt.time_from_start = rclcpp::Duration::from_seconds(std::max(l_time, r_time));

      combined_plan.trajectory_.joint_trajectory.points.push_back(pt);
    }


    size_t nj = combined_plan.trajectory_.joint_trajectory.joint_names.size();
    size_t np = combined_plan.trajectory_.joint_trajectory.points.size();
    RCLCPP_INFO(logger, "Executing combined trajectory (%zu joints, %zu points)", nj, np);
    auto exec_result = mgi_dual.execute(combined_plan);

    bool exec_ok = (exec_result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!exec_ok) {
      RCLCPP_ERROR(logger, "Execution failed!");
    }
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}