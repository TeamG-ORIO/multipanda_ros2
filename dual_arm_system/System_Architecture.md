# Prompt for Coding Agent

# System Overview

The robot performs a **pick, label, and drop task**.

Sequence:

1. Detect item in pickup zone or manual trigger
2. Arm1 picks item from pickup zone
3. Arm1 places item in labeling zone
4. Arm2 picks label from label dispenser
5. Arm2 places label on item
6. Arm1 picks labeled item
7. Arm1 places item in drop zone

For now the system is **strictly sequential**.

Both arms share workspace but **never move simultaneously** in the first implementation.

---

# Architectural Design

The system contains the following logical layers.

```
GUI
   |
Task Supervisor (FSM)
   |
Manipulation Interface
   |
Perception Interface
   |
MoveIt / Motion Planning
```

The **task supervisor orchestrates the sequence**.

The supervisor **does not interact directly with MoveIt or perception algorithms**.

All robot actions must go through the **manipulation interface**.

The manipulation module internally handles:

* motion planning
* gripper control
* grasp execution
* interaction with perception

---

# ROS2 Packages

Create the following packages.

```
dual_arm_system/

    task_supervisor/
    rqt_dual_arm_gui/

    manipulation_interface/
    perception_interface/

    dual_arm_msgs/
    config/
```

Only these packages must contain real functionality:

```
task_supervisor
rqt_dual_arm_gui
```

The other packages should contain **interfaces or stub implementations only**.

---

# Task Supervisor Package

The task supervisor controls the global system state.

Implement it as a **finite state machine**.

## Required States

```
HOME
WAIT_FOR_ITEM
ARM1_PICK
ARM1_PLACE_LABEL_STATION
ARM2_PICK_LABEL
ARM2_PLACE_LABEL
ARM1_PICK_LABELLED_ITEM
ARM1_PLACE_DROP_ZONE
ERROR
```

---

## Home State Behavior

The system always returns to HOME after finishing.

If the system fails, the user should first confirm it is safe to return to HOME.

HOME should:

```
command both arms to go_home()
clear error states
reset retry counters
```

---

## Retry Logic

Failures should be retried.

Retry limits must be configurable via YAML.

Example config:

```
max_grasp_attempts: 3
max_label_attempts: 3
max_perception_attempts: 3
```

If retries are exhausted:

```
transition to ERROR
then return to HOME
notify GUI
```

---

# Manipulation Interface

The manipulation module is responsible for executing robot actions.

It acts as the abstraction layer between the FSM and MoveIt.

The manipulation interface internally calls:

* motion planning (MoveIt)
* gripper control
* perception modules

---

## High Level Task Primitives

These primitives are used by the FSM during normal operation.

Arm1:

```
go_home
pick_item_from_pickup
place_item_in_label_station
pick_labeled_item
place_item_drop_zone
```

Arm2:

```
go_home
pick_label
place_label_on_item
```

These services must return:

```
success
error_message
```

---

## Low Level Motion Primitives (for Testing)

The FSM should also be able to call basic motion commands for testing and debugging.

Expose primitives such as:

```
move_to_pose
activate_gripper
deactivate_gripper
go_home
```

Example usage:

```
move_to_pose(pre_grasp)
move_to_pose(grasp)
activate_gripper
move_to_pose(lift)
```

These primitives must still go through the manipulation interface.

The FSM must **never call MoveIt directly**.

---

## Mock Implementations

For now the manipulation module should provide **mock implementations** that immediately return success.

Real manipulation logic will be implemented later.

---

# Perception Interfaces

Perception will be implemented later.

Create interface definitions for:

Arm1 grasp pose generation

```
get_grasp_pose
```

Arm2 label placement pose generation

```
get_label_pose
```

Define messages that include:

```
pose
pre_grasp_pose
confidence
success
error_message
```

---

## YAML Mock Provider

Provide a mock provider that reads poses from YAML.

Example config:

```
mock_grasp_pose:
  position: [x, y, z]
  orientation: [r, p, y]
```

The orientation in YAML is given in **Euler angles**.

Before publishing the message:

* Convert Euler angles to a quaternion
* Implement a helper function in a utils file to perform this conversion

---

## Provider Switching

The architecture must allow switching perception providers via launch files.

Example:

```
grasp_provider:=yaml
grasp_provider:=vision
```

The default provider should be:

```
yaml
```

If a provider is selected but not implemented, the node should throw a clear ROS error.

---

# GUI Package

Create an **rqt plugin** called:

```
rqt_dual_arm_gui
```

The GUI should provide the following controls.

## Core System Buttons

```
HOME
START SEQUENCE
STOP
RESET ERROR
```

---

## Sequence Selection

The system supports **layered sequences**.

### Mode Selection

```
Complete System
Single Arm
```

If **Single Arm** is selected, the user must select:

```
ARM1
ARM2
```

---

## ARM1 Sequences

```
Pick and Place
Pick Item
Place Item
Activate Gripper
Deactivate Gripper
Pre-Grasp
Pre-Place
```

---

## ARM2 Sequences

```
Pick Label
Place Label
Activate Gripper
Deactivate Gripper
```

The GUI should send the selected sequence request to the **task supervisor**, which will trigger the appropriate FSM transitions.

---

## Display Information

```
Current FSM state
Arm1 state
Arm2 state
Error messages
Retry counters
```

The GUI communicates **only with the task supervisor**.

The GUI must **never call the manipulation interface directly**.

---

# Data Interfaces

Create a message package:

```
dual_arm_msgs
```

Define messages such as:

```
ExecutionResult.msg
GraspPose.msg
LabelPose.msg
```

Failures must be explicit.

Example message structure:

```
bool success
string error_message
```

---

# Config Files

Add configuration files under:

```
config/
```

Example files:

```
retry_config.yaml
mock_poses.yaml
system_modes.yaml
```

These parameters should be loaded by the supervisor node.

---

# System Modes

Support two execution modes.

Discrete mode:

```
user presses start
run one cycle
return HOME
```

Continuous mode:

```
process items continuously
loop sequence
```

---

# Launch Files

Create launch files for:

```
simulation_mode.launch
hardware_mode.launch
```

These launch files should select appropriate providers.

Example:

```
perception_provider:=yaml
manipulation_provider:=mock
```

---

# Code Requirements

Follow these architectural rules:

1. The supervisor must be the **single orchestrator** of the system.
2. The GUI communicates **only with the supervisor**.
3. The FSM interacts with the robot **only through the manipulation interface**.
4. The FSM must **never call MoveIt directly**.
5. Perception modules must be **swappable providers**.
6. All failures must propagate back to the supervisor.

---

