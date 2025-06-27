#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from stair_detection.msg import StairInfo
import time


class ServoAngleController:
    def __init__(self):
        self.rate_hz  = rospy.get_param('~rate', 50)
        self.bounce_s = rospy.get_param('~bounce', 0.02)
        self.hysteresis = rospy.get_param('~hysteresis', 2)
        self.topic = rospy.get_param('~topic', '/servo_commands')

        self.x1 = self.y1 = 92  
        self.last_x = self.last_y = rospy.Time.now()
        self.last_target_angle = None  # Track last target angle


        self.pub = rospy.Publisher(self.topic, Int32MultiArray, queue_size=1)
        rospy.Subscriber('/stair_info', StairInfo, self.stair_info_callback)
        self.rate = rospy.Rate(self.rate_hz)

    def stair_info_callback(self, msg):
        distance = msg.distance
           
       
        # target_angle = (90 - target_angle) 
        # rospy.loginfo("Received log info: distance=%.2f, angle=%d", distance, angle)

         # Only update if the angle changes by more than 2 degrees
        
        if distance < 0.35: 
            angle = int(max(0, min(180, msg.angle)))
            target_angle = int(angle + 0.05)

            if self.last_target_angle is not None and abs(target_angle - self.last_target_angle) < 6:
                rospy.loginfo("Angle change (%.2f) is less than 2 degrees, ignoring.", abs(target_angle - self.last_target_angle))
                return

            self.last_target_angle = target_angle

            if target_angle != 0:

                xp = (-1.141 * target_angle) + 241.7
                yp =  (1.075 * target_angle) +  41.21
                xp = max(0, min(270, xp)); yp = max(0, min(270, yp))
                rospy.loginfo("Target angle: %d, Mapped coordinates: (%.2f, %.2f)", target_angle, xp, yp)

                x2 = int(xp * 180.0 / 270.0)
                y2 = int(yp * 180.0 / 270.0)
                rospy.loginfo("Mapped target: [%d, %d]", x2, y2)

                self.smooth_move(x2, y2)
                
                time.sleep(3)
                target_angle = -30
                p = (-1.141 * target_angle) + 241.7
                yp =  (1.075 * target_angle) +  41.21
                xp = max(0, min(270, xp)); yp = max(0, min(270, yp))
                rospy.loginfo("Target angle: %d, Mapped coordinates: (%.2f, %.2f)", target_angle, xp, yp)

                x2 = int(xp * 180.0 / 270.0)
                y2 = int(yp * 180.0 / 270.0)
                rospy.loginfo("Mapped target: [%d, %d]", x2, y2)

                self.smooth_move(x2, y2)

            # if distance <= 0.2:
            #     rospy.loginfo("Distance is less than 0.2m, moving to position [42, 41]")
            #     self.x1 = 42
            #     self.y1 = 41
            #     self.smooth_move(self.x1, self.y1)

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

            # Clamp safe servo range
            self.x1 = max(3, min(180, self.x1))
            self.y1 = max(5, min(180, self.y1))

            self.pub.publish(Int32MultiArray(data=[self.x1, self.y1]))
            self.rate.sleep()

        rospy.loginfo(f"Reached final position: [{self.x1}, {self.y1}]")


if __name__ == '__main__':
    try:
        rospy.init_node('servo_commands_publisher')
        controller = ServoAngleController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass