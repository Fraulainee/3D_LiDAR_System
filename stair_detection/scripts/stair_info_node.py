#!/usr/bin/env python3

import rospy
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from stair_detection.msg import StairInfo


def stair_info_callback(msg):
    min_step_height = 0.1
    max_step_height = 0.19
    min_step_depth = 0.29

    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    xz_points = [(x, z) for x, y, z in points]

    if len(xz_points) < 10:
        return

    xz_points.sort(key=lambda p: p[0])

    prev_z = xz_points[0][1]
    prev_x = xz_points[0][0]
    angle = 0

    for x, z in xz_points[1:]:
        dz = z - prev_z
        dx = x - prev_x

        if dz >= min_step_height and dz <= max_step_height and dx >= min_step_depth:
            dz = (dz - 0.1265) * 100
            angle = math.degrees(math.atan(dz / x))
            stair_msg = StairInfo()
            stair_msg.distance = x  
            stair_msg.height = dz
            stair_msg.angle = angle
            

            # rospy.loginfo(f"First Stair Detected: Distance={x:.2f} cm Height={dz:.2f} cm Angle={angle:.2f} degrees")
            # rospy.loginfo(f"First Stair Detected: Height={dz:.2f} cm Angle={angle:.2f} degrees")

            
            stair_info_pub.publish(stair_msg)

            return  

        prev_z = z
        prev_x = x


if __name__ == '__main__':
    rospy.init_node('stair_info_node')

    rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_info_callback)
    stair_info_pub = rospy.Publisher('/stair_info', StairInfo, queue_size=1)

    rospy.loginfo("Stair Info Node Started...")
    rospy.spin()
