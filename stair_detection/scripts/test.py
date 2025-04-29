#!/usr/bin/env python3

import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint

def livox_custom_filter_callback(msg):
    threshold_y = 0.2
    min_x = 0.0

    filtered_msg = CustomMsg()
    filtered_msg.header = msg.header
    filtered_msg.timebase = msg.timebase
    filtered_msg.point_num = 0
    filtered_msg.lidar_id = msg.lidar_id

    for pt in msg.points:
        if abs(pt.y) < threshold_y and pt.x > min_x:
            filtered_msg.points.append(pt)

    filtered_msg.point_num = len(filtered_msg.points)
    rospy.loginfo_once(f"Publishing filtered CustomMsg: {filtered_msg.point_num} points")

    custom_pub.publish(filtered_msg)


if __name__ == '__main__':
    rospy.init_node('livox_custom_plane_filter')

    rospy.Subscriber('/livox/lidar', CustomMsg, livox_custom_filter_callback)
    custom_pub = rospy.Publisher('/livox/custom_xz_filtered', CustomMsg, queue_size=1)

    rospy.loginfo("Livox CustomMsg XZ Filter Node Started...")
    rospy.spin()
