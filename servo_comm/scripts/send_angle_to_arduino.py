#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from stair_detection.msg import StairInfo

class ServoAngleController:
    def __init__(self):
        self.rate_hz  = rospy.get_param('~rate', 50)
        self.bounce_s = rospy.get_param('~bounce', 0.02)
        self.hysteresis = rospy.get_param('~hysteresis', 2)
        self.topic = rospy.get_param('~topic', '/servo_commands')

        self.x1 = self.y1 = 92  
        self.last_x = self.last_y = rospy.Time.now()

        self.pub = rospy.Publisher(self.topic, Int32MultiArray, queue_size=1)
        rospy.Subscriber('/stair_info', StairInfo, self.stair_info_callback)
        self.rate = rospy.Rate(self.rate_hz)

    def stair_info_callback(self, msg):
        distance = msg.distance
           
        angle = int(max(0, min(180, msg.angle)))
        target_angle = int(angle + 0.05)
        # target_angle = (90 - target_angle) 
        # rospy.loginfo("Received log info: distance=%.2f, angle=%d", distance, angle)

        if target_angle != 0:
            xp = (1.09667* target_angle) + 40.30
            yp =  (-1.08667* target_angle) +  235.8
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
