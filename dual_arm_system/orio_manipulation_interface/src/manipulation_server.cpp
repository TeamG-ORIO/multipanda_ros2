#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "orio_manipulation_interface/mock_manipulation.hpp"
#include "orio_dual_arm_msgs/srv/arm_command.hpp"
#include "orio_dual_arm_msgs/srv/move_to_pose.hpp"
#include "orio_dual_arm_msgs/srv/gripper_command.hpp"

using namespace orio_manipulation_interface;
using ArmCommand    = orio_dual_arm_msgs::srv::ArmCommand;
using MoveToPose    = orio_dual_arm_msgs::srv::MoveToPose;
using GripperCmd    = orio_dual_arm_msgs::srv::GripperCommand;

class ManipulationServer : public rclcpp::Node
{
public:
  ManipulationServer()
  : Node("manipulation_server")
  {
    // NOTE: backend_ is intentionally NOT constructed here.
    // shared_from_this() is invalid inside a constructor; call init() after
    // the shared_ptr is fully established (see main()).
    arm_cmd_srv_ = create_service<ArmCommand>(
      "~/arm_command",
      [this](const ArmCommand::Request::SharedPtr req,
             ArmCommand::Response::SharedPtr res) {
        handle_arm_command(req, res);
      });

    move_pose_srv_ = create_service<MoveToPose>(
      "~/move_to_pose",
      [this](const MoveToPose::Request::SharedPtr req,
             MoveToPose::Response::SharedPtr res) {
        handle_move_to_pose(req, res);
      });

    gripper_srv_ = create_service<GripperCmd>(
      "~/gripper_command",
      [this](const GripperCmd::Request::SharedPtr req,
             GripperCmd::Response::SharedPtr res) {
        handle_gripper_command(req, res);
      });

  }

  void init()
  {
    backend_ = std::make_shared<MockManipulation>(shared_from_this());
    RCLCPP_INFO(get_logger(), "Manipulation server ready (mock backend).");
  }

private:
  std::shared_ptr<ManipulationInterface> backend_;

  rclcpp::Service<ArmCommand>::SharedPtr  arm_cmd_srv_;
  rclcpp::Service<MoveToPose>::SharedPtr  move_pose_srv_;
  rclcpp::Service<GripperCmd>::SharedPtr  gripper_srv_;

  void handle_arm_command(
    const ArmCommand::Request::SharedPtr req,
    ArmCommand::Response::SharedPtr res)
  {
    ActionResult result;
    const auto & cmd = req->command;

    if (cmd == "go_home") {
      result = backend_->go_home(req->arm_id);
    } else if (cmd == "pick_item_from_pickup") {
      result = backend_->pick_item_from_pickup();
    } else if (cmd == "place_item_in_label_station") {
      result = backend_->place_item_in_label_station();
    } else if (cmd == "pick_labeled_item") {
      result = backend_->pick_labeled_item();
    } else if (cmd == "place_item_drop_zone") {
      result = backend_->place_item_drop_zone();
    } else if (cmd == "pick_label") {
      result = backend_->pick_label();
    } else if (cmd == "place_label_on_item") {
      result = backend_->place_label_on_item();
    } else {
      result = {false, "Unknown command: " + cmd};
      RCLCPP_ERROR(get_logger(), "Unknown arm command: %s", cmd.c_str());
    }

    res->success       = result.success;
    res->error_message = result.error_message;
  }

  void handle_move_to_pose(
    const MoveToPose::Request::SharedPtr req,
    MoveToPose::Response::SharedPtr res)
  {
    auto result = backend_->move_to_pose(req->arm_id, req->named_pose, req->target_pose);
    res->success       = result.success;
    res->error_message = result.error_message;
  }

  void handle_gripper_command(
    const GripperCmd::Request::SharedPtr req,
    GripperCmd::Response::SharedPtr res)
  {
    ActionResult result;
    if (req->activate) {
      result = backend_->activate_gripper(req->arm_id);
    } else {
      result = backend_->deactivate_gripper(req->arm_id);
    }
    res->success       = result.success;
    res->error_message = result.error_message;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulationServer>();
  // init() must be called after make_shared so that shared_from_this() is valid.
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
