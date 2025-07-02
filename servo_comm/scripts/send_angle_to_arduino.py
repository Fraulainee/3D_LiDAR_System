#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32MultiArray
from stair_detection.msg import StairInfo, Attitude

happen_once = True

class ServoAngleController:
    def __init__(self):
        self.rate_hz = rospy.get_param('~rate', 50)
        self.bounce_s = rospy.get_param('~bounce', 0.02)
        self.hysteresis = rospy.get_param('~hysteresis', 2)
        self.topic = rospy.get_param('~topic', '/servo_commands')

        self.x1 = 98
        self.y1 = 84
        self.last_x = self.last_y = rospy.Time.now()

        self.distance = 9999
        self.pitch = 0.0
        self.target_angle = 0
        self.last_target_angle = None

        self.initial_lowered = False

        self.pub = rospy.Publisher(self.topic, Int32MultiArray, queue_size=1)
        rospy.Subscriber('/stair_info', StairInfo, self.stair_info_callback)
        rospy.Subscriber('/pixhawk_attitude', Attitude, self.attitude_callback)

        self.rate = rospy.Rate(self.rate_hz)

    def stair_info_callback(self, msg):
        self.distance = msg.distance
        self.target_angle = int(max(0, min(180, msg.angle)))
        self.process()

    def attitude_callback(self, msg):
        self.pitch = msg.pitch
        self.process()

    def process(self):
        rospy.loginfo(f"[DEBUG] Distance: {self.distance:.3f}, Angle: {self.target_angle}, Pitch: {self.pitch:.2f}")

        if self.distance < 0.35 and not self.initial_lowered:
            rospy.loginfo("[ACTION] Lowering servo due to close obstacle.")
            self.move_to_angle(self.target_angle)
            time.sleep(3)

            rospy.loginfo("[ACTION] Resetting servo to -20 angle.")
            self.reset_position()
            time.sleep(3)

            self.initial_lowered = True
            self.last_target_angle = self.target_angle
            return

        # Ignore until we’ve lowered once
        if not self.initial_lowered:
            return
        
        if self.distance > 900 and self.pitch > 37.0 and not happen_once:
            happen_once = True

            rospy.loginfo("[ACTION] Distance very large and pitch high — moving servo to angle 180.")
            self.move_to_angle(180)
            self.last_target_angle = 180
            return

        # Only proceed if distance and angle are valid and changed significantly
        if self.distance <= 0.0 or self.target_angle == 0:
            return

        if self.last_target_angle is not None and abs(self.target_angle - self.last_target_angle) < 10.0:
            return

        # Process new target angle
        rospy.loginfo(f"[ACTION] Significant change. Moving to angle {self.target_angle}")
        self.move_to_angle(self.target_angle)
        self.last_target_angle = self.target_angle

    def move_to_angle(self, angle):
        xp = (1.076 * angle) + 50.36
        yp = (-1.08 * angle) + 222.2
        xp = max(0, min(270, xp)); yp = max(0, min(270, yp))
        x2 = int(xp * 180.0 / 270.0)
        y2 = int(yp * 180.0 / 270.0)
        self.smooth_move(x2, y2)

    def reset_position(self):
        self.move_to_angle(-20)

    def smooth_move(self, x2, y2):
        while not rospy.is_shutdown() and (self.x1 != x2 or self.y1 != y2):
            now = rospy.Time.now()

            if (now - self.last_x).to_sec() >= self.bounce_s:
                if abs(x2 - self.x1) > self.hysteresis:
                    self.x1 += 1 if x2 > self.x1 else -1
                else:
                    self.x1 = x2
                self.last_x = now

            if (now - self.last_y).to_sec() >= self.bounce_s:
                if abs(y2 - self.y1) > self.hysteresis:
                    self.y1 += 1 if y2 > self.y1 else -1
                else:
                    self.y1 = y2
                self.last_y = now

            self.x1 = max(14, min(180, self.x1))
            self.y1 = max(8, min(168, self.y1))
            self.pub.publish(Int32MultiArray(data=[self.x1, self.y1]))
            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('servo_commands_publisher')
        controller = ServoAngleController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
