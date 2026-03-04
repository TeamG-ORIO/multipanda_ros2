#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace orio_manipulation_interface
{

/// Result returned by every manipulation primitive.
struct ActionResult
{
  bool success{false};
  std::string error_message;
};

/// Abstract base for all manipulation backends (mock, MoveIt, …).
class ManipulationInterface
{
public:
  virtual ~ManipulationInterface() = default;

  // ── High-level task primitives ────────────────────────────────────────────

  /// Drive both arms to their home configuration.
  virtual ActionResult go_home(const std::string & arm_id) = 0;

  // Arm-1 task primitives
  virtual ActionResult pick_item_from_pickup() = 0;
  virtual ActionResult place_item_in_label_station() = 0;
  virtual ActionResult pick_labeled_item() = 0;
  virtual ActionResult place_item_drop_zone() = 0;

  // Arm-2 task primitives
  virtual ActionResult pick_label() = 0;
  virtual ActionResult place_label_on_item() = 0;

  // ── Low-level motion primitives (testing / debugging) ────────────────────

  virtual ActionResult move_to_pose(
    const std::string & arm_id,
    const std::string & named_pose,
    const geometry_msgs::msg::Pose & target_pose) = 0;

  virtual ActionResult activate_gripper(const std::string & arm_id) = 0;
  virtual ActionResult deactivate_gripper(const std::string & arm_id) = 0;
};

}  // namespace orio_manipulation_interface
