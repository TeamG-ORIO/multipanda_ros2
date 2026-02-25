#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

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
  declare_if_not("x",     0.3);
  declare_if_not("y",     0.4);
  declare_if_not("z",     0.4);
  // Euler angles in degrees (ZYX / yaw-pitch-roll convention)
  // Default: 180° around X (end-effector pointing down) = roll:180, pitch:0, yaw:0
  declare_if_not("roll",  180.0);
  declare_if_not("pitch", 0.0);
  declare_if_not("yaw",   0.0);

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

  // Build the MoveIt planning scene from scene.xml objects
  moveit::planning_interface::PlanningSceneInterface psi;

  auto make_box = [](const std::string & id,
                     double px, double py, double pz,
                     double sx, double sy, double sz) {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = "panda_link0";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {sx * 2.0, sy * 2.0, sz * 2.0};  // MuJoCo size = half-extent

    geometry_msgs::msg::Pose pose;
    pose.position.x = px;
    pose.position.y = py;
    pose.position.z = pz;
    pose.orientation.w = 1.0;

    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    return obj;
  };

  std::vector<moveit_msgs::msg::CollisionObject> scene_objects;

  // Table: body pos=(0,0,-0.584), geom half-sizes=(0.343,0.546,0.584)
  scene_objects.push_back(make_box("table",       0.0,    0.0,   -0.584,  0.343, 0.546, 0.584));

  // Frame posts and crossbar
  scene_objects.push_back(make_box("left_post",   0.0,  -0.597,  0.5715, 0.02,  0.02,  0.5715));
  scene_objects.push_back(make_box("right_post",  0.0,   0.597,  0.5715, 0.02,  0.02,  0.5715));
  scene_objects.push_back(make_box("crossbar",    0.0,   0.0,    1.143,  0.02,  0.597, 0.02));

  // Graspable object (obj_box_01): body pos=(-0.3,0.3,0.03), half-sizes=(0.03,0.03,0.03)
  scene_objects.push_back(make_box("obj_box_01", -0.3,   0.3,    0.03,   0.03,  0.03,  0.03));

  psi.applyCollisionObjects(scene_objects);

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