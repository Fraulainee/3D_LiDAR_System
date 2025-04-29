#!/usr/bin/env python3

import rospy
import math
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def stair_detection_callback(msg):
    # Step 1: Read the points
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    xz_points = [(x, y, z) for x, y, z in points]

    if len(xz_points) < 10:
        stair_pub.publish(Bool(data=False))
        return

    # Step 2: Convert to Open3D format
    pc_array = np.array(xz_points, dtype=np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_array)

    # Step 3: Build KD-Tree
    kd_tree = o3d.geometry.KDTreeFlann(pcd)

    # Step 4: Manual density filtering using KD-Tree
    search_radius = 0.1
    min_neighbors = 10
    filtered_indices = []

    for i, point in enumerate(pcd.points):
        [k, idx, _] = kd_tree.search_radius_vector_3d(point, search_radius)
        if k >= min_neighbors:
            filtered_indices.append(i)

    if len(filtered_indices) < 10:
        rospy.logwarn("Too few dense points after filtering.")
        stair_pub.publish(Bool(data=False))
        return

    # Keep only dense points
    pcd = pcd.select_by_index(filtered_indices)

    # Step 5: Estimate Normals (still needed)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30)
    )
    pcd.normalize_normals()

    # Step 6: (Optional) Downsample
    voxel_size = 0.03
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Step 7: Initialize Marker for stairs
    stair_count = 0

    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "stairs"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    marker.pose.orientation.w = 1.0

    # Step 8: Plane Segmentation (RANSAC)
    while len(pcd.points) > 30:
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.02,
            ransac_n=3,
            num_iterations=1000
        )

        [a, b, c, d] = plane_model

        # Only vertical planes
        if abs(a) < 0.8 and abs(c) > 0.5:
            stair_count += 1

            inlier_cloud = pcd.select_by_index(inliers)
            inlier_points = np.asarray(inlier_cloud.points)

            for pt in inlier_points:
                p1 = Point(pt[0], 0.0, pt[2])
                p2 = Point(pt[0], 0.0, pt[2] + 0.1)
                marker.points.append(p1)
                marker.points.append(p2)

        # Remove inliers for next iteration
        pcd = pcd.select_by_index(inliers, invert=True)

    # Step 9: Publish Results
    if len(marker.points) > 0:
        marker_pub.publish(marker)
    else:
        rospy.logwarn("No valid stair planes detected. Empty marker not published.")

    if stair_count >= 2:
        rospy.loginfo(f"STAIRS DETECTED: {stair_count} risers found.")
        stair_pub.publish(Bool(data=True))
    else:
        stair_pub.publish(Bool(data=False))


if __name__ == '__main__':
    rospy.init_node('stair_detector_node_kdtree_filtered')

    rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_detection_callback)

    stair_pub = rospy.Publisher('/stairs_detected', Bool, queue_size=1)
    marker_pub = rospy.Publisher('/stair_markers', Marker, queue_size=1)

    rospy.loginfo("Stair Detection Node Started (KD-Tree Filtered + RANSAC)...")
    rospy.spin()
