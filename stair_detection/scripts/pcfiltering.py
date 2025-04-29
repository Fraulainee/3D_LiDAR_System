#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def pointcloud_callback(msg):
    threshold_x = 0.0
    threshold_y = 0.2

    filtered_points = []

    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = p

        if x > threshold_x and abs(y) < threshold_y:
            filtered_points.append([x, y, z])

    # rospy.loginfo_once(f"Filtered {len(filtered_points)} points!")
    
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id

    filtermsg = pc2.create_cloud_xyz32(header, filtered_points)

    filter.publish(filtermsg)



if __name__ == '__main__':
    rospy.init_node('pointcloud_filter_node')

    filter = rospy.Publisher('/livox/pcfilter', PointCloud2, queue_size=1)

    rospy.Subscriber('/livox/xz_plane', PointCloud2, pointcloud_callback)

    rospy.spin()
