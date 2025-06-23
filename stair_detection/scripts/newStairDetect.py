#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def pointcloud2_to_xyz_array(msg):
    return np.array([
        [x, y, z] for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ])

def stair_detection_callback(msg):
    points_np = pointcloud2_to_xyz_array(msg)
    if len(points_np) < 30:
        stair_pub.publish(Bool(data=False))
        return

    # Convert to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)

    # Remove ground via RANSAC
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    if abs(c) > 0.9:
        pcd = pcd.select_by_index(inliers, invert=True)

    filtered_xyz = np.asarray(pcd.points)
    xz_points = [(x, z) for x, y, z in filtered_xyz]
    if len(xz_points) < 10:
        stair_pub.publish(Bool(data=False))
        return

    xz_points.sort(key=lambda p: p[0])

    # Estimate average dz and dx
    dz_list, dx_list = [], []
    prev_x, prev_z = xz_points[0]
    for x, z in xz_points[1:]:
        dz = z - prev_z
        dx = x - prev_x
        if dz > 0 and dx > 0:
            dz_list.append(dz)
            dx_list.append(dx)
        prev_x, prev_z = x, z

    if len(dz_list) < 2:
        stair_pub.publish(Bool(data=False))
        return

    avg_dz = np.mean(dz_list)
    avg_dx = np.mean(dx_list)
    rospy.loginfo(f"Avg dz: {avg_dz:.3f}, Avg dx: {avg_dx:.3f}")

    # Reset for step detection
    prev_x, prev_z = xz_points[0]
    step_count = 0

    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "stairs"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.03
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0

    for x, z in xz_points[1:]:
        dz = z - prev_z
        dx = x - prev_x

        if (0.7 * avg_dz) <= dz <= (1.3 * avg_dz) and dx >= (0.7 * avg_dx):
            step_count += 1
            p1 = Point(x, 0, prev_z)
            p2 = Point(x, 0, z)
            marker.points.append(p1)
            marker.points.append(p2)
            prev_z, prev_x = z, x

    if marker.points:
        marker_pub.publish(marker)

    stair_pub.publish(Bool(data=(step_count >= 2)))

if __name__ == '__main__':
    rospy.init_node('adaptive_stair_detector_with_ransac')
    rospy.Subscriber('/livox/xz_downsampled', PointCloud2, stair_detection_callback)
    stair_pub = rospy.Publisher('/stairs_detected', Bool, queue_size=1)
    marker_pub = rospy.Publisher('/stair_markers', Marker, queue_size=1)
    rospy.loginfo("Stair Detection Node (Dynamic with RANSAC) Started...")
    rospy.spin()
