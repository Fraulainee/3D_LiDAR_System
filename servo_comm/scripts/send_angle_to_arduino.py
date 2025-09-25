#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32MultiArray
from stair_detection.msg import StairInfo, Attitude

class ServoAngleController:
    def __init__(self):
        self.rate_hz = rospy.get_param('~rate', 50)
        self.bounce_s = rospy.get_param('~bounce', 0.02)
        self.hysteresis = rospy.get_param('~hysteresis', 2)
        self.topic = rospy.get_param('~topic', '/servo_commands')

        self.x1 = 98
        self.y1 = 84
        self.last_x = self.last_y = rospy.Time.now()

        self.target_x = self.x1
        self.target_y = self.y1

        self.distance = 9999
        self.pitch = 0.0
        self.target_angle = 0
        self.last_target_angle = None

        self.initial_lowered = False
        self.happen_once = False
        self.is_descending = False

        self.pub = rospy.Publisher(self.topic, Int32MultiArray, queue_size=1)
        rospy.Subscriber('/log_info', StairInfo, self.stair_info_callback)
        rospy.Subscriber('/pixhawk_attitude', Attitude, self.attitude_callback)

        self.rate = rospy.Rate(self.rate_hz)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_callback)

    def stair_info_callback(self, msg):
        self.distance = msg.distance
        self.target_angle = int(max(0, min(180, msg.angle)))
        # rospy.loginfo(f"[DEBUG] Received distance: {self.distance:.2f}, target angle: {self.target_angle}")

    def attitude_callback(self, msg):
        self.pitch = msg.pitch
        # rospy.loginfo(f"[DEBUG] Received pitch: {self.pitch:.2f}")

    def timer_callback(self, event):
        self.process()
        self.incremental_servo_move()

    def process(self):
        # rospy.loginfo(f"[DEBUG] Processing with distance: {self.distance:.2f}, pitch: {self.pitch:.2f}, target angle: {self.target_angle}")
        
        if self.distance < 0.33 and not self.initial_lowered:
            rospy.loginfo("[ACTION] Lowering servo due to close obstacle.")
            self.set_target_angle(self.target_angle)
            rospy.sleep(3)

            rospy.loginfo("[ACTION] Resetting servo to -20 angle.")
            self.set_target_angle(-25)
            # rospy.sleep(3)

            self.initial_lowered = True
            self.last_target_angle = self.target_angle
            return

        if not self.initial_lowered:
            return
        
        # if self.distance > 900 and -0.5 < self.pitch < 7 and self.is_descending:
        #     rospy.loginfo("[ACTION] Move flippers forward")
        #     self.set_target_angle(0)
        #     self.last_target_angle = 0
        #     return

        if self.distance > 900 and self.pitch >= 28.0 and not self.happen_once: # for 14cm use 28
            self.happen_once = True
            rospy.loginfo("[ACTION] Distance very large and pitch high — moving servo to angle 205.")
            self.set_target_angle(205)
            self.last_target_angle = 205
            # self.is_descending = True
            return

        if self.distance < 1.0 and  -17.0 < self.pitch < -9.0: # for 14cm use -15
            rospy.loginfo("[ACTION] Descending, moving servo to angle 180.")
            self.set_target_angle(90)
            self.last_target_angle = 90
            return
        
        if 0.35 < self.distance < 0.8 and self.pitch < -18.0: # for 14cm use -18
            rospy.loginfo("[ACTION] Slowly touching ground.")
            self.set_target_angle(174)
            self.last_target_angle = 174
            return

        if self.distance <= 0.0 or self.target_angle == 0:
            return

        if self.last_target_angle is not None and abs(self.target_angle - self.last_target_angle) < 10.0:
            return

        # rospy.loginfo(f"[ACTION] Significant change. Moving to angle {self.target_angle}")
        # self.set_target_angle(self.target_angle)
        # self.last_target_angle = self.target_angle

    def set_target_angle(self, angle):
        xp = (1.09778 * angle) + 35.76
        yp = (-1.10476 * angle) + 229.36
        xp = max(0, min(270, xp))
        yp = max(0, min(270, yp))
        self.target_x = int(xp * 180.0 / 270.0)
        self.target_y = int(yp * 180.0 / 270.0)

    def incremental_servo_move(self):
        now = rospy.Time.now()

        moved = False
        if (now - self.last_x).to_sec() >= self.bounce_s:
            if abs(self.target_x - self.x1) > self.hysteresis:
                self.x1 += 1 if self.target_x > self.x1 else -1
                moved = True
            else:
                self.x1 = self.target_x
            self.last_x = now

        if (now - self.last_y).to_sec() >= self.bounce_s:
            if abs(self.target_y - self.y1) > self.hysteresis:
                self.y1 += 1 if self.target_y > self.y1 else -1
                moved = True
            else:
                self.y1 = self.target_y
            self.last_y = now

        self.x1 = max(14, min(180, self.x1))
        self.y1 = max(8, min(168, self.y1))

        if moved:
            self.pub.publish(Int32MultiArray(data=[self.x1, self.y1]))

if __name__ == '__main__':
    try:
        rospy.init_node('servo_commands_publisher')
        controller = ServoAngleController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
