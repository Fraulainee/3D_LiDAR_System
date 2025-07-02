#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32MultiArray

class ManualServoController:
    def __init__(self):
        rospy.init_node('manual_angle_publisher', anonymous=True)
        self.pub = rospy.Publisher('/servo_commands', Int32MultiArray, queue_size=1)

        self.x1 = 98  
        self.y1 = 100 
        self.rate = rospy.Rate(50)
        self.bounce_s = 0.02
        self.hysteresis = 2

        self.last_x = self.last_y = rospy.Time.now()

    def map_angle(self, angle):
        xp = (1.076 * angle) + 50.36
        yp = (-1.08 * angle) + 222.2
        xp = max(0, min(270, xp))
        yp = max(0, min(270, yp))
        x_servo = int(xp * 180.0 / 270.0)
        y_servo = int(yp * 180.0 / 270.0)
        return x_servo, y_servo

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

    def run(self):
        print("Manual Servo Angle Publisher with Smooth Movement")
        print("Enter an angle between 0–180. Ctrl+C to exit.")
        while not rospy.is_shutdown():
            try:
                angle_input = int(input("Target Angle: "))
                angle_input = max(0, min(180, angle_input))
                x2, y2 = self.map_angle(angle_input)
                rospy.loginfo(f"Moving to x: {x2}, y: {y2} from [{self.x1}, {self.y1}]")
                self.smooth_move(x2, y2)
            except ValueError:
                print("Invalid input. Please enter an integer between 0 and 180.")
            except rospy.ROSInterruptException:
                break

if __name__ == '__main__':
    controller = ManualServoController()
    controller.run()
