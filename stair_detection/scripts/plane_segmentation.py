#!/usr/bin/env python3

# import rospy
# import math
# from livox_ros_driver2.msg import CustomMsg
# from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
# import std_msgs.msg

# # last_pub_time = rospy.Time(0)

# def livox_callback(msg):
#     # global last_pub_time
#     threshold = 0.2
#     filtered_points = []

#     now = rospy.Time.now()

#     for pt in msg.points:
#         x, y, z = pt.x, pt.y, pt.z

#         if abs(y) < threshold and 0 < x < 1.0:
#             filtered_points.append([x, y, z])

#     header = std_msgs.msg.Header()
#     header.stamp = rospy.Time.now()
#     header.frame_id = msg.header.frame_id  

#     # if (now - last_pub_time).to_sec() >= 1.0:  
#     cloud_msg = pc2.create_cloud_xyz32(header, filtered_points)
#     pub.publish(cloud_msg)
#         # last_pub_time = now


# if __name__ == '__main__':
#     rospy.init_node('livox_xz_plane_node')

#     pub = rospy.Publisher('/livox/xz_plane', PointCloud2, queue_size=1)

#     rospy.Subscriber('/livox/lidar', CustomMsg, livox_callback)
#     rospy.loginfo("Listening to /livox/lidar - Filtering XZ Plane Front Points...")

#     rospy.spin()



import rospy
import math
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

# last_pub_time = rospy.Time(0)

def livox_callback(msg):
    global last_pub_time
    threshold = 0.10
    filtered_points = []

    now = rospy.Time.now()

    for pt in msg.points:
        x, y, z = pt.x, pt.y, pt.z
        # x = -(x)
        # z = -(z)
        intensity = float(pt.reflectivity)
        # filtered_points.append([x, y, z, intensity])
        
        

        if abs(y) < threshold and 1.0 > x > 0.05 and z < 0.2:
        # if 1.0 > y > 0.1 and z < 0.25 and abs(x) < threshold:
            filtered_points.append([x, y, z, intensity])

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]

    # if (now - last_pub_time).to_sec() >= 0.5:
    cloud_msg = pc2.create_cloud(header, fields, filtered_points)
    pub.publish(cloud_msg)
        # last_pub_time = now


if __name__ == '__main__':
    rospy.init_node('livox_xz_plane_node')
    last_pub_time = rospy.Time.now()

    pub = rospy.Publisher('/livox/xz_plane', PointCloud2, queue_size=1)
    rospy.Subscriber('/livox/lidar', CustomMsg, livox_callback)
    rospy.loginfo("Listening to /livox/lidar - Filtering XZ Plane Front Points...")
    rospy.spin()
