"""Task Supervisor Node.

Finite-state machine that orchestrates the dual-arm pick-label-place
sequence.  This node is the single orchestrator:

  * The GUI sends commands here (SupervisorCommand service).
  * All robot actions go through the manipulation interface.
  * FSM state is published on /supervisor/status.

FSM state diagram (happy path):

  HOME
   └─ START ──> WAIT_FOR_ITEM
                  └─> ARM1_PICK
                        └─> ARM1_PLACE_LABEL_STATION
                              └─> ARM2_PICK_LABEL
                                    └─> ARM2_PLACE_LABEL
                                          └─> ARM1_PICK_LABELLED_ITEM
                                                └─> ARM1_PLACE_DROP_ZONE
                                                      └─> HOME  (discrete)
                                                      └─> WAIT_FOR_ITEM  (continuous)

Errors:
  any state ──failure──> retry up to N times ──exhausted──> ERROR ──> HOME
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from orio_dual_arm_msgs.srv import SupervisorCommand, ArmCommand
from orio_dual_arm_msgs.msg import SystemStatus

from orio_task_supervisor.states import SystemState, ExecutionMode
from orio_task_supervisor.retry_manager import RetryManager


class TaskSupervisorNode(Node):

    def __init__(self):
        super().__init__('task_supervisor')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('max_grasp_attempts', 3)
        self.declare_parameter('max_label_attempts', 3)
        self.declare_parameter('max_perception_attempts', 3)
        self.declare_parameter('execution_mode', 'discrete')
        self.declare_parameter('status_publish_hz', 2.0)

        max_grasp       = self.get_parameter('max_grasp_attempts').value
        max_label       = self.get_parameter('max_label_attempts').value
        max_perception  = self.get_parameter('max_perception_attempts').value
        mode_str        = self.get_parameter('execution_mode').value
        pub_hz          = self.get_parameter('status_publish_hz').value

        self._mode = (ExecutionMode.CONTINUOUS
                      if mode_str == 'continuous' else ExecutionMode.DISCRETE)
        self._retries = RetryManager(max_grasp, max_label, max_perception)

        # ── State ─────────────────────────────────────────────────────────────
        self._state = SystemState.HOME
        self._arm1_state = 'idle'
        self._arm2_state = 'idle'
        self._error_message = ''
        self._running = False        # set True when a sequence is active
        self._stop_requested = False # set by STOP command mid-sequence
        self._pending_home_confirm = False  # waiting for user to confirm HOME after ERROR

        # ── ROS interfaces ────────────────────────────────────────────────────
        cb = ReentrantCallbackGroup()

        self._cmd_srv = self.create_service(
            SupervisorCommand,
            'supervisor/command',
            self._handle_command,
            callback_group=cb)

        self._status_pub = self.create_publisher(
            SystemStatus, 'supervisor/status', 10)

        self._manip_arm_cmd = self.create_client(
            ArmCommand, 'manipulation_server/arm_command')

        self._status_timer = self.create_timer(
            1.0 / pub_hz, self._publish_status)

        self.get_logger().info(
            f'Task supervisor ready | mode={self._mode.value} | '
            f'max_grasp={max_grasp} max_label={max_label} '
            f'max_perception={max_perception}')

    # ── Command handler ───────────────────────────────────────────────────────

    def _handle_command(
        self,
        request: SupervisorCommand.Request,
        response: SupervisorCommand.Response,
    ) -> SupervisorCommand.Response:

        cmd = request.command.lower().strip()
        self.get_logger().info(
            f'Command received: cmd={cmd!r} seq={request.sequence!r} '
            f'arm={request.arm_id!r}')

        if cmd == 'home':
            response.success, response.error_message = self._cmd_home()
        elif cmd == 'start_sequence':
            response.success, response.error_message = self._cmd_start(
                request.sequence, request.arm_id)
        elif cmd == 'stop':
            response.success, response.error_message = self._cmd_stop()
        elif cmd == 'reset_error':
            response.success, response.error_message = self._cmd_reset_error()
        else:
            response.success = False
            response.error_message = f"Unknown command: '{cmd}'"
            self.get_logger().warn(response.error_message)

        return response

    # ── Top-level commands ────────────────────────────────────────────────────

    def _cmd_home(self):
        if self._state == SystemState.ERROR and not self._pending_home_confirm:
            return False, (
                'System is in ERROR state.  Confirm it is safe to move, '
                'then call reset_error before commanding HOME.')
        self._transition_to(SystemState.HOME)
        self._execute_home()
        return True, ''

    def _cmd_start(self, sequence: str, arm_id: str):
        if self._running:
            return False, 'Sequence already running.'
        if self._state == SystemState.ERROR:
            return False, 'Cannot start: system is in ERROR state. Call reset_error first.'

        seq = sequence.lower().strip() if sequence else 'complete'
        arm = arm_id.lower().strip() if arm_id else ''

        self._stop_requested = False
        self._running = True

        if seq == 'complete' or seq == '':
            self.get_logger().info('Starting complete pick-label-place sequence.')
            self.create_timer(0.0, self._run_full_sequence, autostart=True)
        elif seq == 'pick_and_place':
            self.get_logger().info(f'Starting pick-and-place for {arm}.')
            self.create_timer(0.0,
                lambda: self._run_single_arm_sequence(arm, 'pick_and_place'),
                autostart=True)
        elif seq in ('pick_item', 'place_item', 'activate_gripper',
                     'deactivate_gripper', 'pre_grasp', 'pre_place',
                     'pick_label', 'place_label'):
            self.get_logger().info(f'Starting primitive sequence: {seq} on {arm}')
            self.create_timer(0.0,
                lambda: self._run_primitive(arm, seq), autostart=True)
        else:
            self._running = False
            return False, f"Unknown sequence: '{sequence}'"

        return True, ''

    def _cmd_stop(self):
        self._stop_requested = True
        self.get_logger().info('STOP requested — sequence will halt after current step.')
        return True, ''

    def _cmd_reset_error(self):
        if self._state != SystemState.ERROR:
            return False, 'System is not in ERROR state.'
        self._pending_home_confirm = True
        self._error_message = ''
        self.get_logger().info(
            'Error acknowledged. Call home to return arms to home position.')
        return True, ''

    # ── FSM sequences ─────────────────────────────────────────────────────────

    def _run_full_sequence(self):
        """Execute the complete pick-label-place cycle."""
        steps = [
            (SystemState.WAIT_FOR_ITEM,            None,             None),
            (SystemState.ARM1_PICK,                'arm1', 'pick_item_from_pickup'),
            (SystemState.ARM1_PLACE_LABEL_STATION, 'arm1', 'place_item_in_label_station'),
            (SystemState.ARM2_PICK_LABEL,          'arm2', 'pick_label'),
            (SystemState.ARM2_PLACE_LABEL,         'arm2', 'place_label_on_item'),
            (SystemState.ARM1_PICK_LABELLED_ITEM,  'arm1', 'pick_labeled_item'),
            (SystemState.ARM1_PLACE_DROP_ZONE,     'arm1', 'place_item_drop_zone'),
        ]

        for state, arm_id, command in steps:
            if self._stop_requested:
                self.get_logger().info('Sequence stopped by user request.')
                break

            self._transition_to(state)

            if command is None:
                # WAIT_FOR_ITEM: in this implementation just proceed immediately.
                self.get_logger().info('Waiting for item — proceeding (auto-trigger).')
                continue

            retry_category = self._retry_category_for(state)
            self._retries.reset(retry_category)

            success = False
            while not success:
                success = self._call_arm_command(arm_id, command)
                if not success:
                    exhausted = self._retries.increment(retry_category)
                    self.get_logger().warn(
                        f'Step {state} failed '
                        f'(attempt {self._retries.count(retry_category)}/'
                        f'{self._retries._limits[retry_category]}).')
                    if exhausted:
                        self._enter_error(
                            f'Max retries exhausted for state {state}.')
                        self._execute_home()
                        self._running = False
                        return

        if not self._stop_requested:
            if self._mode == ExecutionMode.DISCRETE:
                self._execute_home()
            else:
                self.get_logger().info('Continuous mode: restarting sequence.')
                self._run_full_sequence()
                return

        self._running = False

    def _run_single_arm_sequence(self, arm_id: str, sequence: str):
        """Execute a sub-sequence for a single arm."""
        if arm_id == 'arm1':
            if sequence == 'pick_and_place':
                commands = [
                    ('arm1', 'pick_item_from_pickup'),
                    ('arm1', 'place_item_drop_zone'),
                ]
            else:
                commands = []
        elif arm_id == 'arm2':
            if sequence == 'pick_and_place':
                commands = [
                    ('arm2', 'pick_label'),
                    ('arm2', 'place_label_on_item'),
                ]
            else:
                commands = []
        else:
            self.get_logger().error(f'Unknown arm_id: {arm_id}')
            self._running = False
            return

        for arm, cmd in commands:
            if self._stop_requested:
                break
            ok = self._call_arm_command(arm, cmd)
            if not ok:
                self._enter_error(f'Single-arm sequence failed at {cmd}')
                self._running = False
                return

        self._execute_home()
        self._running = False

    def _run_primitive(self, arm_id: str, primitive: str):
        """Execute a single low-level primitive."""
        ok = self._call_arm_command(arm_id, primitive)
        if not ok:
            self._enter_error(f'Primitive {primitive} failed on {arm_id}.')
        self._running = False

    # ── Home / error helpers ───────────────────────────────────────────────────

    def _execute_home(self):
        self._transition_to(SystemState.HOME)
        self._retries.reset()
        self._arm1_state = 'homing'
        self._arm2_state = 'homing'
        self._call_arm_command('arm1', 'go_home')
        self._call_arm_command('arm2', 'go_home')
        self._arm1_state = 'idle'
        self._arm2_state = 'idle'
        self._pending_home_confirm = False
        self.get_logger().info('Both arms returned HOME.')

    def _enter_error(self, message: str):
        self._error_message = message
        self._transition_to(SystemState.ERROR)
        self.get_logger().error(f'ERROR: {message}')

    def _transition_to(self, new_state: SystemState):
        self.get_logger().info(
            f'FSM transition: {self._state} -> {new_state}')
        self._state = new_state

    # ── Manipulation client ───────────────────────────────────────────────────

    def _call_arm_command(self, arm_id: str, command: str) -> bool:
        """Send an ArmCommand to the manipulation server, return success."""
        if not self._manip_arm_cmd.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                'Manipulation server not available (arm_command service).')
            return False

        req = ArmCommand.Request()
        req.arm_id  = arm_id
        req.command = command

        future = self._manip_arm_cmd.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error(
                f'arm_command({arm_id}, {command}) timed out.')
            return False

        result = future.result()
        if not result.success:
            self.get_logger().error(
                f'arm_command({arm_id}, {command}) failed: {result.error_message}')
        return result.success

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self):
        msg = SystemStatus()
        msg.fsm_state             = str(self._state)
        msg.arm1_state            = self._arm1_state
        msg.arm2_state            = self._arm2_state
        msg.error_message         = self._error_message
        msg.grasp_retry_count     = self._retries.count('grasp')
        msg.label_retry_count     = self._retries.count('label')
        msg.perception_retry_count = self._retries.count('perception')
        self._status_pub.publish(msg)

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _retry_category_for(state: SystemState) -> str:
        """Map an FSM state to a retry category."""
        grasp_states = {
            SystemState.ARM1_PICK,
            SystemState.ARM1_PICK_LABELLED_ITEM,
            SystemState.ARM2_PICK_LABEL,
        }
        label_states = {
            SystemState.ARM2_PLACE_LABEL,
            SystemState.ARM1_PLACE_LABEL_STATION,
            SystemState.ARM1_PLACE_DROP_ZONE,
        }
        if state in grasp_states:
            return 'grasp'
        if state in label_states:
            return 'label'
        return 'perception'


def main(args=None):
    rclpy.init(args=args)
    node = TaskSupervisorNode()
    rclpy.spin(node)
    rclpy.shutdown()
