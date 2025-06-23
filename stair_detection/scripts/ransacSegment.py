#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def segment_planes_callback(msg):
    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if len(points) < 50:
        return

    cloud_np = np.array(points, dtype=np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_np)

    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "segmented_planes"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0

    while len(pcd.points) > 50:
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.03,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        [a, b, c, d] = plane_model
        normal = np.array([a, b, c])
        inlier_cloud = pcd.select_by_index(inliers)
        inlier_np = np.asarray(inlier_cloud.points)

        if abs(c) > 0.9:
            rospy.loginfo("Detected horizontal plane (tread/floor), skipping.")
        elif abs(c) < 0.3:
            rospy.loginfo("Detected vertical plane (riser).")
            for pt in inlier_np:
                p1 = Point(pt[0], 0.0, pt[2])
                p2 = Point(pt[0], 0.0, pt[2] + 0.1)
                marker.points.append(p1)
                marker.points.append(p2) 
        else:
            rospy.loginfo("Discarding angled plane.")

        # Remove this plane's points for next iteration
        pcd = pcd.select_by_index(inliers, invert=True)

    if marker.points:
        marker_pub.publish(marker)
    else:
        rospy.logwarn("No marker points detected after segmentation.")

if __name__ == '__main__':
    rospy.init_node('ransac_plane_segmenter')

    rospy.Subscriber('/livox/xz_plane', PointCloud2, segment_planes_callback)
    marker_pub = rospy.Publisher('/stair_segment_markers', Marker, queue_size=1)

    rospy.loginfo("RANSAC Plane Segmentation Node Running...")
    rospy.spin()
