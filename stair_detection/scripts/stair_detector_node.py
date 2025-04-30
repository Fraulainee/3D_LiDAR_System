#!/usr/bin/env python3

# import rospy
# import math
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Bool
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point


# def stair_detection_callback(msg):
#     min_step_height = 0.1
#     max_step_height = 0.19
#     min_step_depth = 0.29

#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     xz_points = [(x, z) for x, y, z in points]

#     if len(xz_points) < 10:
#         stair_pub.publish(Bool(data=False))
#         return

#     xz_points.sort(key=lambda p: p[0])

#     step_count = 0
#     prev_x, prev_z = xz_points[0]

#     # Line marker for all detected risers
#     marker = Marker()
#     marker.header.frame_id = msg.header.frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "stairs"
#     marker.id = 0
#     marker.type = Marker.LINE_LIST
#     marker.action = Marker.ADD
#     marker.scale.x = 0.05
#     marker.color.a = 1.0
#     marker.color.r = 1.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0

#     for x, z in xz_points[1:]:
#         dz = z - prev_z
#         dx = x - prev_x

#         if min_step_height <= dz <= max_step_height and dx >= min_step_depth:
#             step_count += 1

#             # Add line marker
#             p1 = Point(x, 0, prev_z)
#             p2 = Point(x, 0, z)
#             marker.points.append(p1)
#             marker.points.append(p2)

#             # Add text marker for height label
#             text_marker = Marker()
#             text_marker.header.frame_id = msg.header.frame_id
#             text_marker.header.stamp = rospy.Time.now()
#             text_marker.ns = "step_text"
#             text_marker.id = step_count
#             text_marker.type = Marker.TEXT_VIEW_FACING
#             text_marker.action = Marker.ADD
#             text_marker.pose.position.x = x
#             text_marker.pose.position.y = 0
#             text_marker.pose.position.z = z + 0.1
#             text_marker.scale.z = 0.2
#             text_marker.color.r = 1.0
#             text_marker.color.g = 1.0
#             text_marker.color.b = 1.0
#             text_marker.color.a = 1.0
#             text_marker.text = f"{abs(dz):.2f} m"

#             marker_text_pub.publish(text_marker)

#             prev_z = z
#             prev_x = x

#     marker_pub.publish(marker)

#     if step_count >= 2:
#         stair_pub.publish(Bool(data=True))
#     else:
#         stair_pub.publish(Bool(data=False))


# if __name__ == '__main__':
#     rospy.init_node('stair_detector_node')

#     rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_detection_callback)

#     stair_pub = rospy.Publisher('/stairs_detected', Bool, queue_size=1)
#     marker_pub = rospy.Publisher('/stair_markers', Marker, queue_size=1)
#     marker_text_pub = rospy.Publisher('/step_text_markers', Marker, queue_size=10)

#     rospy.loginfo("Stair Detection Node Started...")
#     rospy.spin()



# import rospy
# import math
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Bool
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point


# def stair_detection_callback(msg):
#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     xz_points = [(x, z) for x, y, z in points]

#     if len(xz_points) < 10:
#         stair_pub.publish(Bool(data=False))
#         return

#     xz_points.sort(key=lambda p: p[0])

#     baseline_dx = None
#     step_count = 0

#     prev_x, prev_z = xz_points[0]

#     marker = Marker()
#     marker.header.frame_id = msg.header.frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "stairs"
#     marker.id = 0
#     marker.type = Marker.LINE_LIST
#     marker.action = Marker.ADD
#     marker.scale.x = 0.05
#     marker.color.a = 1.0
#     marker.color.r = 1.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0

#     for x, z in xz_points[1:]:
#         dz = z - prev_z
#         dx = x - prev_x

#         if dz <= 0 or dx <= 0:
#             prev_x = x
#             prev_z = z
#             continue

#         if baseline_dx is None:
#             # First detected step (by height)
#             if 0.05 <= dz <= 0.4 and dx >= 0.1:
#                 baseline_dx = dx
#                 rospy.loginfo(f"[Baseline Set] dx={baseline_dx:.3f} m (step depth)")

#                 p1 = Point(x, 0, prev_z)
#                 p2 = Point(x, 0, z)
#                 marker.points.append(p1)
#                 marker.points.append(p2)

#                 step_count += 1

#         else:
#             # After baseline: only check dx (depth between steps)
#             dx_min = baseline_dx * 0.8
#             dx_max = baseline_dx * 1.2

#             if dx_min <= dx <= dx_max:
#                 p1 = Point(x, 0, prev_z)
#                 p2 = Point(x, 0, z)
#                 marker.points.append(p1)
#                 marker.points.append(p2)

#                 step_count += 1

#         prev_x = x
#         prev_z = z

#     marker_pub.publish(marker)

#     if step_count >= 2:
#         rospy.loginfo(f"STAIRS DETECTED: {step_count} steps.")
#         stair_pub.publish(Bool(data=True))
#     else:
#         stair_pub.publish(Bool(data=False))


# if __name__ == '__main__':
#     rospy.init_node('stair_detector_node')

#     rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_detection_callback)

#     stair_pub = rospy.Publisher('/stairs_detected', Bool, queue_size=1)
#     marker_pub = rospy.Publisher('/stair_markers', Marker, queue_size=1)

#     rospy.loginfo("Stair Detection Node Started (Depth-based)...")
#     rospy.spin()


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

    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    xz_points = [(x, y, z) for x, y, z in points]

    if len(xz_points) < 10:
        stair_pub.publish(Bool(data=False))
        return


    # Convert to Open3D format
    pc_array = np.array(xz_points, dtype=np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_array)

    stair_count = 0

    # Marker initialization
    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "stairs"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # Fix orientation
    marker.pose.orientation.w = 1.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0

    # Plane segmentation loop
    while len(pcd.points) > 30:
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.06,
            ransac_n=3,
            num_iterations=1000
        )

        [a, b, c, d] = plane_model
        # print(plane_model)

        if abs(a) < 0.8 and abs(c) > 0.5:
            stair_count += 1

            inlier_cloud = pcd.select_by_index(inliers)
            inlier_points = np.asarray(inlier_cloud.points)

            for pt in inlier_points:
                p1 = Point(pt[0], 0.0, pt[2])
                p2 = Point(pt[0], 0.0, pt[2] + 0.1)
                marker.points.append(p1)
                marker.points.append(p2)

        # Remove the detected plane from point cloud
        pcd = pcd.select_by_index(inliers, invert=True)

    # Only publish marker if points exist
    if len(marker.points) > 0:
        marker_pub.publish(marker)
    else:
        rospy.logwarn("No valid stair planes detected. Empty marker not published.")

    # Publish stair detection status
    if stair_count >= 2:
        # rospy.loginfo(f"STAIRS DETECTED: {stair_count} risers found.")
        stair_pub.publish(Bool(data=True))
    else:
        # rospy.loginfo("No stairs detected in this frame.")
        stair_pub.publish(Bool(data=False))


if __name__ == '__main__':
    rospy.init_node('stair_detector_node_ransac')

    rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_detection_callback)

    stair_pub = rospy.Publisher('/stairs_detected', Bool, queue_size=1)
    marker_pub = rospy.Publisher('/stair_markers', Marker, queue_size=1)

    # rospy.loginfo("Stair Detection Node Started (RANSAC-based, Safe Publishing)...")
    rospy.spin()
