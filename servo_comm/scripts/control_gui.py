#!/usr/bin/env python3
import sys, rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import (
    QApplication, QSlider, QVBoxLayout, QWidget,
    QLabel, QPushButton, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer

# Servo angle range constants
MIN_ANGLE = -30
MAX_ANGLE = 210
HOME_ANGLE = 90

class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        # Initialize ROS node and publishers
        rospy.init_node('robot_control_gui', anonymous=True)
        self.servo_pub = rospy.Publisher('/servo_commands', Int32MultiArray, queue_size=1)
        self.cmd_pub   = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Ensure focus for key events
        self.setFocusPolicy(Qt.StrongFocus)

        # Servo A label and slider
        self.lblA = QLabel(f"Servo A/B: {HOME_ANGLE}°")
        self.sldrA = QSlider(Qt.Horizontal)
        self.sldrA.setRange(MIN_ANGLE, MAX_ANGLE)
        self.sldrA.setTickPosition(QSlider.TicksBelow)
        self.sldrA.setTickInterval(20)
        self.sldrA.setValue(HOME_ANGLE)
        self.sldrA.valueChanged.connect(self.on_sldr_changed)

        # Drive control buttons
        self.fwd   = QPushButton("Forward")
        self.bwd   = QPushButton("Backward")
        self.left  = QPushButton("Turn Left")
        self.right = QPushButton("Turn Right")
        for btn, twist in [
            (self.fwd,  (1, 0)), (self.bwd, (-1, 0)),
            (self.left, (0, 1)), (self.right, (0, -1))
        ]:
            btn.pressed.connect(lambda t=twist: self.drive(t[0], t[1]))
            btn.released.connect(lambda: self.drive(0, 0))

        # Home button
        self.home_btn = QPushButton("Home")
        self.home_btn.clicked.connect(self.reset_position)

        # Layout
        layout = QVBoxLayout(self)
        layout.addWidget(self.lblA)
        layout.addWidget(self.sldrA)
        btn_layout = QHBoxLayout()
        for btn in (self.home_btn, self.fwd, self.bwd, self.left, self.right):
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)
        self.setLayout(layout)
        self.setWindowTitle("Robot Control Panel")

        # ROS spin timer
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rospy.spin_once())
        self.timer.start(50)

    def on_sldr_changed(self, value):
        # Update label
        self.lblA.setText(f"Servo A/B: {value}°")
        # Compute inverse for B
        inv = MIN_ANGLE + MAX_ANGLE - value
        # Publish A and inverted B
        self.servo_pub.publish(Int32MultiArray(data=[value, inv]))

    def reset_position(self):
        self.sldrA.blockSignals(True)
        self.sldrA.setValue(HOME_ANGLE)
        self.sldrA.blockSignals(False)
        self.on_sldr_changed(HOME_ANGLE)

    def drive(self, lin, ang):
        t = Twist()
        t.linear.x  = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = ControlGUI()
    gui.show()
    sys.exit(app.exec_())
