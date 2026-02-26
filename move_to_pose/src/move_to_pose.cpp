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
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose
  auto const target_pose = [&]{
    geometry_msgs::msg::Pose msg;

    constexpr double DEG2RAD = M_PI / 180.0;
    double roll  = node->get_parameter("roll").as_double()  * DEG2RAD;
    double pitch = node->get_parameter("pitch").as_double() * DEG2RAD;
    double yaw   = node->get_parameter("yaw").as_double()   * DEG2RAD;

    // ZYX Euler (yaw * pitch * roll) -> quaternion
    double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);
    double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);

    msg.orientation.w = cr * cp * cy + sr * sp * sy;
    msg.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.orientation.z = cr * cp * sy - sr * sp * cy;
    msg.position.x = node->get_parameter("x").as_double();
    msg.position.y = node->get_parameter("y").as_double();
    msg.position.z = node->get_parameter("z").as_double();
    return msg;
  }();

  move_group_interface.setPoseTarget(target_pose);

  const std::string frame = move_group_interface.getPlanningFrame();

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  // panda_link0 is at world z=1 (orio_panda.xml pos="0 0 1"), so all positions
  // below are relative to the panda_link0 frame: world_z - 1.0.

  // --- scene.xml: table (body pos="0 0 0.5", geom size="0.4 0.6 0.5") ---
  // world center=(0,0,0.5), panda frame z=0.5-1.0=-0.5
  collision_objects.push_back(make_box(
    "table", frame,
    {0.0, 0.0, -0.5},
    {0.4, 0.6, 0.5}));

  // --- scene.xml: frame body pos="0 0 1", left_post geom pos="0 -0.597 0.5", size="0.02 0.02 0.5" ---
  // world center=(0,-0.597,1.5), panda frame z=1.5-1.0=0.5
  collision_objects.push_back(make_box(
    "left_post", frame,
    {0.0, -0.597, 0.5},
    {0.02, 0.02, 0.5}));

  // --- scene.xml: frame body pos="0 0 1", right_post geom pos="0 0.597 0.5", size="0.02 0.02 0.5" ---
  // world center=(0,0.597,1.5), panda frame z=1.5-1.0=0.5
  collision_objects.push_back(make_box(
    "right_post", frame,
    {0.0, 0.597, 0.5},
    {0.02, 0.02, 0.5}));

  // --- scene.xml: frame body pos="0 0 1", crossbar geom pos="0 0 1.02", size="0.02 0.6 0.02" ---
  // world center=(0,0,2.02), panda frame z=2.02-1.0=1.02
  collision_objects.push_back(make_box(
    "crossbar", frame,
    {0.0, 0.0, 1.02},
    {0.02, 0.6, 0.02}));

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(collision_objects);
  RCLCPP_INFO(logger, "Planning scene populated with %zu collision objects.", collision_objects.size());

  // Give move_group time to process the scene update before planning.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Verify the objects actually landed in the scene before planning.
  {
    auto known = planning_scene_interface.getKnownObjectNames();
    RCLCPP_INFO(logger, "Objects confirmed in scene: %zu", known.size());
    for (const auto & name : known) {
      RCLCPP_INFO(logger, "  - %s", name.c_str());
    }
    if (known.size() != collision_objects.size()) {
      RCLCPP_WARN(logger, "Scene object count mismatch — waiting another 500 ms");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }


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