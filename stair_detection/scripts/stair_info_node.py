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
            dz = (dz - 0.10) 
            dz = abs(dz) * 100
            x = abs(dx) * 100
            x = math.sqrt(x**2 - dz**2)

            angle = math.degrees(math.atan(dz / x))

            # Store values for averaging
            if not hasattr(stair_info_callback, "data_buffer"):
                stair_info_callback.data_buffer = []

            stair_info_callback.data_buffer.append((x, dz, angle))

            # Keep only the last 15 data points
            if len(stair_info_callback.data_buffer) > 15:
                stair_info_callback.data_buffer.pop(0)

            # Calculate averages
            if len(stair_info_callback.data_buffer) == 15:
                avg_x = sum(item[0] for item in stair_info_callback.data_buffer) / 15
                avg_dz = sum(item[1] for item in stair_info_callback.data_buffer) / 15
                avg_angle = sum(item[2] for item in stair_info_callback.data_buffer) / 15

                stair_msg = StairInfo()
                stair_msg.distance = avg_x  
                stair_msg.height = avg_dz
                stair_msg.angle = avg_angle

                rospy.loginfo(f"Averaged Stair Info: Distance={avg_x:.2f} cm Height={avg_dz:.2f} cm Angle={avg_angle:.2f} degrees")
                stair_info_pub.publish(stair_msg)

        prev_z = z
        prev_x = x


if __name__ == '__main__':
    rospy.init_node('stair_info_node')

    rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_info_callback)
    stair_info_pub = rospy.Publisher('/stair_info', StairInfo, queue_size=1)

    rospy.loginfo("Stair Info Node Started...")
    rospy.spin()
