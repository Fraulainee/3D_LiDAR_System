#!/usr/bin/env python3

import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

def livox_callback(msg):
    filtered_points = []
    for pt in msg.points:
        x, y, z = pt.x, pt.y, pt.z 

        if abs(y)<= 0.5 and abs(z) <= 0.5:
            filtered_points.append([x,y,z])

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id
    
    cloud_msg = pc2.create_cloud_xyz32(header, filtered_points)
    publisher_name.publish(cloud_msg)



if __name__ == '__main__':
    rospy.init_node('node_name')
    publisher_name = rospy.Publisher('topic_name', PointCloud2, queue_size=1)
    rospy.Subscriber('/livox/lidar', CustomMsg, livox_callback)
    rospy.spin()
