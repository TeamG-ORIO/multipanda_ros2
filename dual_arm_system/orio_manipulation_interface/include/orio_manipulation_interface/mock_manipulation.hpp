#pragma once

#include "orio_manipulation_interface/manipulation_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace orio_manipulation_interface
{

/// Mock backend: every call logs the action and immediately returns success.
/// Replace this class (or add a new derived class) when MoveIt is integrated.
class MockManipulation : public ManipulationInterface
{
public:
  explicit MockManipulation(rclcpp::Node::SharedPtr node)
  : node_(node), logger_(node->get_logger()) {}

  ActionResult go_home(const std::string & arm_id) override
  {
    RCLCPP_INFO(logger_, "[MOCK] go_home(%s)", arm_id.c_str());
    return {true, ""};
  }

  ActionResult pick_item_from_pickup() override
  {
    RCLCPP_INFO(logger_, "[MOCK] pick_item_from_pickup()");
    return {true, ""};
  }

  ActionResult place_item_in_label_station() override
  {
    RCLCPP_INFO(logger_, "[MOCK] place_item_in_label_station()");
    return {true, ""};
  }

  ActionResult pick_labeled_item() override
  {
    RCLCPP_INFO(logger_, "[MOCK] pick_labeled_item()");
    return {true, ""};
  }

  ActionResult place_item_drop_zone() override
  {
    RCLCPP_INFO(logger_, "[MOCK] place_item_drop_zone()");
    return {true, ""};
  }

  ActionResult pick_label() override
  {
    RCLCPP_INFO(logger_, "[MOCK] pick_label()");
    return {true, ""};
  }

  ActionResult place_label_on_item() override
  {
    RCLCPP_INFO(logger_, "[MOCK] place_label_on_item()");
    return {true, ""};
  }

  ActionResult move_to_pose(
    const std::string & arm_id,
    const std::string & named_pose,
    const geometry_msgs::msg::Pose & /*target_pose*/) override
  {
    if (!named_pose.empty()) {
      RCLCPP_INFO(logger_, "[MOCK] move_to_pose(%s, named=%s)", arm_id.c_str(), named_pose.c_str());
    } else {
      RCLCPP_INFO(logger_, "[MOCK] move_to_pose(%s, explicit_pose)", arm_id.c_str());
    }
    return {true, ""};
  }

  ActionResult activate_gripper(const std::string & arm_id) override
  {
    RCLCPP_INFO(logger_, "[MOCK] activate_gripper(%s)", arm_id.c_str());
    return {true, ""};
  }

  ActionResult deactivate_gripper(const std::string & arm_id) override
  {
    RCLCPP_INFO(logger_, "[MOCK] deactivate_gripper(%s)", arm_id.c_str());
    return {true, ""};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
};

}  // namespace orio_manipulation_interface
