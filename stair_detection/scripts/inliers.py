#!/usr/bin/env python3

import rospy
import math
import numpy as np
import open3d as o3d

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2


def o3d_to_ros_pointcloud(pcd, frame_id="map"):
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    data = []

    for i in range(len(points)):
        x, y, z = points[i]
        r, g, b = (colors[i] * 255).astype(np.uint8)
        rgb = (r << 16) | (g << 8) | b
        data.append([x, y, z, rgb])

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.UINT32, 1),
    ]

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    return pc2.create_cloud(header, fields, data)


def pointcloud_callback(msg):
    # Convert PointCloud2 to numpy array
    points = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])

    if len(points) == 0:
        return

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))

    # Run RANSAC to find planes (stairs)
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model

    # Filter based on plane normal
    if abs(a) < 0.8 and abs(c) > 0.5:
        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])  # Red

        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        outlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])  # Gray

        # Convert to ROS messages
        inlier_msg = o3d_to_ros_pointcloud(inlier_cloud, frame_id=msg.header.frame_id)
        outlier_msg = o3d_to_ros_pointcloud(outlier_cloud, frame_id=msg.header.frame_id)

        # Publish
        inlier_pub.publish(inlier_msg)
        outlier_pub.publish(outlier_msg)


if __name__ == "__main__":
    rospy.init_node("stair_ransac_publisher")

    inlier_pub = rospy.Publisher("/stair_inliers", PointCloud2, queue_size=1)
    outlier_pub = rospy.Publisher("/stair_outliers", PointCloud2, queue_size=1)

    rospy.Subscriber("/livox/xz_plane", PointCloud2, pointcloud_callback)

    rospy.loginfo("Stair RANSAC Node Running - Visualizing Inliers and Outliers")
    rospy.spin()
