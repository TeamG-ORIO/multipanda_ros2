"""rqt plugin: Dual Arm GUI.

All robot interactions are routed exclusively through the task supervisor.
This widget never calls the manipulation interface directly.

Layout:
  ┌──────────────────────────────────────────────────┐
  │  [HOME]  [START SEQUENCE]  [STOP]  [RESET ERROR] │
  ├──────────────────────────────────────────────────┤
  │  Mode: (●) Complete System  ( ) Single Arm       │
  │         └─ Arm:  [ARM1 ▾]                        │
  │  Sequence: [Pick and Place ▾]                    │
  ├──────────────────────────────────────────────────┤
  │  FSM State:   HOME          Arm1: idle           │
  │  Arm2: idle   Retries G:0 L:0 P:0                │
  │  Error: —                                        │
  └──────────────────────────────────────────────────┘
"""

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QRadioButton,
    QButtonGroup, QSizePolicy,
)

from qt_gui.plugin import Plugin

import rclpy
from rclpy.node import Node

from orio_dual_arm_msgs.srv import SupervisorCommand
from orio_dual_arm_msgs.msg import SystemStatus


# ── Sequence definitions ──────────────────────────────────────────────────────

_ARM1_SEQUENCES = [
    'Pick and Place',
    'Pick Item',
    'Place Item',
    'Activate Gripper',
    'Deactivate Gripper',
    'Pre-Grasp',
    'Pre-Place',
]

_ARM2_SEQUENCES = [
    'Pick Label',
    'Place Label',
    'Activate Gripper',
    'Deactivate Gripper',
]

_SEQUENCE_TO_CMD = {
    'Pick and Place':      'pick_and_place',
    'Pick Item':           'pick_item',
    'Place Item':          'place_item',
    'Activate Gripper':    'activate_gripper',
    'Deactivate Gripper':  'deactivate_gripper',
    'Pre-Grasp':           'pre_grasp',
    'Pre-Place':           'pre_place',
    'Pick Label':          'pick_label',
    'Place Label':         'place_label',
}


class DualArmGui(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('DualArmGui')

        self._node: Node = context.node

        self._cmd_client = self._node.create_client(
            SupervisorCommand, 'supervisor/command')

        self._status_sub = self._node.create_subscription(
            SystemStatus,
            'supervisor/status',
            self._on_status,
            10)

        self._widget = QWidget()
        self._widget.setWindowTitle('Dual Arm Control')
        self._build_ui()

        context.add_widget(self._widget)

        # Poll rclpy spin to process incoming messages
        self._spin_timer = QTimer(self._widget)
        self._spin_timer.timeout.connect(self._ros_spin_once)
        self._spin_timer.start(50)  # 20 Hz

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout()
        self._widget.setLayout(root)

        root.addWidget(self._build_system_buttons())
        root.addWidget(self._build_sequence_selector())
        root.addWidget(self._build_status_display())
        root.addStretch()

    def _build_system_buttons(self) -> QGroupBox:
        group = QGroupBox('System Control')
        layout = QHBoxLayout()
        group.setLayout(layout)

        self._btn_home = QPushButton('HOME')
        self._btn_start = QPushButton('START SEQUENCE')
        self._btn_stop = QPushButton('STOP')
        self._btn_reset = QPushButton('RESET ERROR')

        for btn in (self._btn_home, self._btn_start, self._btn_stop, self._btn_reset):
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            layout.addWidget(btn)

        self._btn_home.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold;')
        self._btn_start.setStyleSheet('background-color: #2196F3; color: white; font-weight: bold;')
        self._btn_stop.setStyleSheet('background-color: #f44336; color: white; font-weight: bold;')
        self._btn_reset.setStyleSheet('background-color: #FF9800; color: white; font-weight: bold;')

        self._btn_home.clicked.connect(lambda: self._send_command('home'))
        self._btn_start.clicked.connect(self._on_start_clicked)
        self._btn_stop.clicked.connect(lambda: self._send_command('stop'))
        self._btn_reset.clicked.connect(lambda: self._send_command('reset_error'))

        return group

    def _build_sequence_selector(self) -> QGroupBox:
        group = QGroupBox('Sequence Selection')
        layout = QVBoxLayout()
        group.setLayout(layout)

        # Mode radio buttons
        mode_layout = QHBoxLayout()
        self._rb_complete = QRadioButton('Complete System')
        self._rb_single   = QRadioButton('Single Arm')
        self._rb_complete.setChecked(True)
        self._mode_group = QButtonGroup()
        self._mode_group.addButton(self._rb_complete)
        self._mode_group.addButton(self._rb_single)
        mode_layout.addWidget(self._rb_complete)
        mode_layout.addWidget(self._rb_single)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)

        # Arm selector (only shown for Single Arm mode)
        arm_layout = QHBoxLayout()
        arm_layout.addWidget(QLabel('Arm:'))
        self._cb_arm = QComboBox()
        self._cb_arm.addItems(['ARM1', 'ARM2'])
        arm_layout.addWidget(self._cb_arm)
        arm_layout.addStretch()
        self._arm_selector_widget = QWidget()
        self._arm_selector_widget.setLayout(arm_layout)
        self._arm_selector_widget.setVisible(False)
        layout.addWidget(self._arm_selector_widget)

        # Sequence combo
        seq_layout = QHBoxLayout()
        seq_layout.addWidget(QLabel('Sequence:'))
        self._cb_sequence = QComboBox()
        seq_layout.addWidget(self._cb_sequence)
        seq_layout.addStretch()
        layout.addLayout(seq_layout)

        # Wire signals
        self._rb_complete.toggled.connect(self._on_mode_changed)
        self._rb_single.toggled.connect(self._on_mode_changed)
        self._cb_arm.currentTextChanged.connect(self._refresh_sequences)

        self._refresh_sequences()
        return group

    def _build_status_display(self) -> QGroupBox:
        group = QGroupBox('System Status')
        layout = QVBoxLayout()
        group.setLayout(layout)

        row1 = QHBoxLayout()
        self._lbl_fsm   = QLabel('FSM State:  HOME')
        self._lbl_arm1  = QLabel('Arm1: idle')
        self._lbl_arm2  = QLabel('Arm2: idle')
        row1.addWidget(self._lbl_fsm)
        row1.addStretch()
        row1.addWidget(self._lbl_arm1)
        row1.addWidget(self._lbl_arm2)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        self._lbl_retries = QLabel('Retries — G: 0  L: 0  P: 0')
        row2.addWidget(self._lbl_retries)
        row2.addStretch()
        layout.addLayout(row2)

        self._lbl_error = QLabel('Error: —')
        self._lbl_error.setStyleSheet('color: #f44336;')
        layout.addWidget(self._lbl_error)

        return group

    # ── Signal handlers ───────────────────────────────────────────────────────

    def _on_mode_changed(self):
        single = self._rb_single.isChecked()
        self._arm_selector_widget.setVisible(single)
        self._refresh_sequences()

    def _refresh_sequences(self):
        self._cb_sequence.clear()
        if self._rb_complete.isChecked():
            self._cb_sequence.addItem('Complete System')
        else:
            arm = self._cb_arm.currentText()
            seqs = _ARM1_SEQUENCES if arm == 'ARM1' else _ARM2_SEQUENCES
            self._cb_sequence.addItems(seqs)

    def _on_start_clicked(self):
        sequence = ''
        arm_id   = ''

        if self._rb_single.isChecked():
            display_seq = self._cb_sequence.currentText()
            sequence    = _SEQUENCE_TO_CMD.get(display_seq, display_seq.lower().replace(' ', '_'))
            arm_id      = self._cb_arm.currentText().lower()
        # complete system: leave sequence and arm_id empty

        self._send_command('start_sequence', sequence=sequence, arm_id=arm_id)

    def _on_status(self, msg: SystemStatus):
        """Update status labels — called from ROS subscription thread."""
        # Posting to the Qt main thread via a lambda and QTimer.singleShot
        # is the safest cross-thread approach in rqt.
        fsm   = msg.fsm_state
        arm1  = msg.arm1_state
        arm2  = msg.arm2_state
        error = msg.error_message or '—'
        g     = msg.grasp_retry_count
        la    = msg.label_retry_count
        p     = msg.perception_retry_count

        # Use a closure to capture current values
        def _update(f=fsm, a1=arm1, a2=arm2, e=error, gv=g, lv=la, pv=p):
            self._lbl_fsm.setText(f'FSM State:  {f}')
            self._lbl_arm1.setText(f'Arm1: {a1}')
            self._lbl_arm2.setText(f'Arm2: {a2}')
            self._lbl_retries.setText(f'Retries — G: {gv}  L: {lv}  P: {pv}')
            self._lbl_error.setText(f'Error: {e}')
            # Highlight error state
            if f == 'ERROR':
                self._lbl_fsm.setStyleSheet('color: #f44336; font-weight: bold;')
            else:
                self._lbl_fsm.setStyleSheet('')

        QTimer.singleShot(0, _update)

    # ── ROS communication ─────────────────────────────────────────────────────

    def _send_command(self, command: str, sequence: str = '', arm_id: str = ''):
        if not self._cmd_client.wait_for_service(timeout_sec=0.5):
            self._node.get_logger().warn(
                'Supervisor command service not available.')
            return

        req = SupervisorCommand.Request()
        req.command  = command
        req.sequence = sequence
        req.arm_id   = arm_id

        future = self._cmd_client.call_async(req)
        future.add_done_callback(self._on_command_response)

    def _on_command_response(self, future):
        try:
            result = future.result()
            if not result.success:
                self._node.get_logger().warn(
                    f'Command failed: {result.error_message}')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'Service call failed: {exc}')

    def _ros_spin_once(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)

    # ── Plugin lifecycle ──────────────────────────────────────────────────────

    def shutdown_plugin(self):
        self._spin_timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('mode', 'single' if self._rb_single.isChecked() else 'complete')
        instance_settings.set_value('arm', self._cb_arm.currentText())

    def restore_settings(self, plugin_settings, instance_settings):
        mode = instance_settings.value('mode', 'complete')
        if mode == 'single':
            self._rb_single.setChecked(True)
        arm = instance_settings.value('arm', 'ARM1')
        idx = self._cb_arm.findText(arm)
        if idx >= 0:
            self._cb_arm.setCurrentIndex(idx)


def main():
    """Stand-alone entry point (without rqt framework)."""
    import sys
    from python_qt_binding.QtWidgets import QApplication

    rclpy.init()
    app = QApplication(sys.argv)

    node = rclpy.create_node('dual_arm_gui')

    class _FakeContext:
        def add_widget(self, w):
            w.show()

        @property
        def node(self):
            return node

    gui = DualArmGui(_FakeContext())
    sys.exit(app.exec_())
