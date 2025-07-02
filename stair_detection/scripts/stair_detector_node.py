#!/usr/bin/env python3

import rospy
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from stair_detection.msg import StairInfo

last_pub_time = rospy.Time(0)
log_height_buffer = []
BUFFER_SIZE = 20
active_log_height = 0.0  
defdistance = 1000
defHeight = 0.0
defAngle = 0.0

def log_detection_callback(msg):
    global last_pub_time
    now = rospy.Time.now()
    global active_log_height
    global log_height_buffer

    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    xz_points = [(x, z) for x, y, z in points if x > 0]

    if len(xz_points) < 10:
        log_pub.publish(Bool(data=False))
        if (now - last_pub_time).to_sec() >= 2.0:
            stair_msg = StairInfo()
            stair_msg.distance = defdistance
            stair_msg.height = defHeight
            stair_msg.angle = defAngle
            stair_info_pub.publish(stair_msg)
            last_pub_time = now
        return

    # Convert to numpy array for processing
    xz_np = np.array(xz_points)

    # Detect local elevated region (log candidate)
    z_median = np.median(xz_np[:, 1])
    z_thresh_min = z_median - 0.02  # reduced from 0.15
    z_thresh_max = z_median + 0.04  # asymmetric range helps detect upward bumps

    log_candidates = xz_np[(xz_np[:, 1] >= z_thresh_min) & (xz_np[:, 1] <= z_thresh_max)]

    if len(log_candidates) < 10:  # reduced threshold for sparse detection
        log_pub.publish(Bool(data=False))
        return

    # Calculate object properties
    center_x = np.mean(log_candidates[:, 0])
    base_z = np.min(log_candidates[:, 1])
    top_z = np.max(log_candidates[:, 1])
    log_height = top_z - base_z + 0.09
    distance = math.sqrt(center_x**2 + base_z**2)

    if log_height < 0.015:  # reject almost-flat surfaces
        log_pub.publish(Bool(data=False))
        return

    log_height_buffer.append(log_height)
    if len(log_height_buffer) > BUFFER_SIZE:
        log_height_buffer.pop(0)

    max_recent_height = max(log_height_buffer)

    angle_deg = math.degrees(math.atan2(max_recent_height, distance))
    rospy.loginfo(f"Log detected: height = {max_recent_height:.3f} m, distance = {distance:.3f} m, angle = {angle_deg:.2f} degrees")

    if (now - last_pub_time).to_sec() >= 2.0:
        stair_msg = StairInfo()
        stair_msg.distance = distance
        stair_msg.height = max_recent_height
        stair_msg.angle = angle_deg
        stair_info_pub.publish(stair_msg)
        last_pub_time = now

    # Marker for RViz
    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "log_marker"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.03
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 1.0

    p1 = Point(center_x, 0.0, base_z)
    p2 = Point(center_x, 0.0, top_z)
    marker.points.append(p1)
    marker.points.append(p2)

    marker_pub.publish(marker)
    log_pub.publish(Bool(data=True))



# def log_detection_callback(msg):
#     global last_pub_time
#     global log_height_buffer

#     now = rospy.Time.now()

#     # Extract y and z values for points in right-forward narrow view
#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     yz_points = [(y, z) for x, y, z in points if 1.0 > y > 0.1 and z < 0.5 and 0.05 > x > 0]

#     if len(yz_points) < 5:
#         log_pub.publish(Bool(data=False))
#         return

#     yz_np = np.array(yz_points)

#     # Direct height measurement using Z bounds
#     z_min = np.min(yz_np[:, 1])
#     z_max = np.max(yz_np[:, 1])
#     log_height = z_max - z_min

#     # Only accept if height is at least 5cm
#     if log_height < 0.05:
#         log_pub.publish(Bool(data=False))
#         return

#     # Optional smoothing: store last 20 log heights
#     log_height_buffer.append(log_height)
#     if len(log_height_buffer) > BUFFER_SIZE:
#         log_height_buffer.pop(0)

#     max_recent_height = max(log_height_buffer)

#     center_y = np.mean(yz_np[:, 0])
#     distance = center_y
#     angle_deg = math.degrees(math.atan2(max_recent_height, distance))

#     rospy.loginfo(f"Obstacle detected: height = {max_recent_height:.3f} m, distance = {distance:.3f} m, angle = {angle_deg:.2f}°")

#     # Publish stair message
#     if (now - last_pub_time).to_sec() >= 2.0:
#         stair_msg = StairInfo()
#         stair_msg.distance = distance
#         stair_msg.height = max_recent_height
#         stair_msg.angle = angle_deg
#         stair_info_pub.publish(stair_msg)
#         last_pub_time = now

#     # Publish RViz Marker (vertical line at detected obstacle)
#     marker = Marker()
#     marker.header.frame_id = msg.header.frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "log_marker"
#     marker.id = 0
#     marker.type = Marker.LINE_LIST
#     marker.action = Marker.ADD
#     marker.scale.x = 0.03  # line width
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 1.0
#     marker.color.a = 1.0
#     marker.pose.orientation.w = 1.0

#     # Line from base_z to top_z at center_y (y-axis)
#     p1 = Point()
#     p1.x = 0.025  # x midpoint in range (0, 0.05)
#     p1.y = center_y
#     p1.z = z_min

#     p2 = Point()
#     p2.x = 0.025
#     p2.y = center_y
#     p2.z = z_max

#     marker.points.append(p1)
#     marker.points.append(p2)

#     marker_pub.publish(marker)
#     log_pub.publish(Bool(data=True))



if __name__ == '__main__':
    rospy.init_node('log_detector_node')

    rospy.Subscriber('/livox/xz_plane', PointCloud2, log_detection_callback)

    log_pub = rospy.Publisher('/log_detected', Bool, queue_size=1)
    marker_pub = rospy.Publisher('/log_marker', Marker, queue_size=1)
    stair_info_pub = rospy.Publisher('/stair_info', StairInfo, queue_size=1)

    rospy.loginfo("Log Detection Node Started...")
    rospy.spin()


# import rospy
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Bool
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point
# import numpy as np

# def log_detection_callback(msg):
#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     xz_points = [(x, z) for x, y, z in points if x > 0]

#     if len(xz_points) < 10:
#         log_pub.publish(Bool(data=False))
#         return

#     # Convert to numpy array for filtering
#     xz_np = np.array(xz_points)

#     # Estimate bounding box of the elevated log object
#     z_median = np.median(xz_np[:, 1])
#     z_thresh_min = z_median - 0.05
#     z_thresh_max = z_median + 0.05

#     log_candidates = xz_np[(xz_np[:, 1] >= z_thresh_min) & (xz_np[:, 1] <= z_thresh_max)]

#     if len(log_candidates) < 20:
#         log_pub.publish(Bool(data=False))
#         return

#     # Calculate log center and height
#     center_x = np.mean(log_candidates[:, 0])
#     base_z = np.min(log_candidates[:, 1])
#     top_z = np.max(log_candidates[:, 1])
#     log_height = top_z - base_z + 0.03

#     rospy.loginfo(f"Log detected: height = {log_height:.3f} m, center_x = {center_x:.3f}")

#     # Create vertical marker to visualize log
#     marker = Marker()
#     marker.header.frame_id = msg.header.frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "log_marker"
#     marker.id = 0
#     marker.type = Marker.LINE_LIST
#     marker.action = Marker.ADD
#     marker.scale.x = 0.03
#     marker.color.a = 1.0
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 1.0
#     marker.pose.orientation.w = 1.0

#     p1 = Point(center_x, 0.0, base_z)
#     p2 = Point(center_x, 0.0, top_z)
#     marker.points.append(p1)
#     marker.points.append(p2)

#     marker_pub.publish(marker)
#     log_pub.publish(Bool(data=True))


# if __name__ == '__main__':
#     rospy.init_node('log_detector_node')

#     rospy.Subscriber('/livox/xz_plane', PointCloud2, log_detection_callback)

#     log_pub = rospy.Publisher('/log_detected', Bool, queue_size=1)
#     marker_pub = rospy.Publisher('/log_marker', Marker, queue_size=1)

#     rospy.loginfo("Log Detection Node Started...")
#     rospy.spin()


# import rospy
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Bool
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point
# import numpy as np

# def log_detection_callback(msg):
#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     xz_points = [(x, z) for x, y, z in points if x > 0]

#     if len(xz_points) < 10:
#         log_pub.publish(Bool(data=False))
#         return

#     # Convert to numpy array for filtering
#     xz_np = np.array(xz_points)

#     # Estimate bounding box of the elevated log object
#     z_median = np.median(xz_np[:, 1])
#     z_thresh_min = z_median - 0.05
#     z_thresh_max = z_median + 0.05

#     log_candidates = xz_np[(xz_np[:, 1] >= z_thresh_min) & (xz_np[:, 1] <= z_thresh_max)]

#     if len(log_candidates) < 20:
#         log_pub.publish(Bool(data=False))
#         return

#     # Calculate log center
#     center_x = np.mean(log_candidates[:, 0])
#     base_z = np.min(log_candidates[:, 1])
#     top_z = np.max(log_candidates[:, 1])

#     # Create vertical marker to visualize log
#     marker = Marker()
#     marker.header.frame_id = msg.header.frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "log_marker"
#     marker.id = 0
#     marker.type = Marker.LINE_LIST
#     marker.action = Marker.ADD
#     marker.scale.x = 0.03
#     marker.color.a = 1.0
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 1.0
#     marker.pose.orientation.w = 1.0

#     p1 = Point(center_x, 0.0, base_z)
#     p2 = Point(center_x, 0.0, top_z)
#     marker.points.append(p1)
#     marker.points.append(p2)

#     marker_pub.publish(marker)
#     log_pub.publish(Bool(data=True))


# if __name__ == '__main__':
#     rospy.init_node('log_detector_node')

#     rospy.Subscriber('/livox/xz_plane', PointCloud2, log_detection_callback)

#     log_pub = rospy.Publisher('/log_detected', Bool, queue_size=1)
#     marker_pub = rospy.Publisher('/log_marker', Marker, queue_size=1)

#     rospy.loginfo("Log Detection Node Started...")
#     rospy.spin()


# import rospy
# import math
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Bool
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point


# def stair_detection_callback(msg):
#     min_step_height = 0.9
#     max_step_height = 0.12
#     min_step_depth = 0.10

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
#     marker.scale.x = 0.02
#     marker.color.a = 1.0
#     marker.color.r = 1.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.pose.orientation.w = 1.0
    
#     for x, z in xz_points[1:]:
#         dz = z - prev_z
#         dx = x - prev_x

#         # rospy.loginfo(f"dx={dx:.4f} m, dz={dz:.4f} m z ={z:.4f} m x={x:.4f} m")

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
# from std_msgs.msg import ColorRGBA
# from std_msgs.msg import Bool
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point


# def stair_detection_callback(msg):
#     min_step_height = 0.1
#     max_step_height = 0.19
#     min_step_depth = 0.29

#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     xz_points = [(x, y, z) for x, y, z in points]

#     if len(xz_points) < 10:
#         return

#     xz_points.sort(key=lambda p: p[0])

#     step_count = 0
#     prev_x, prev_z = xz_points[0][0], xz_points[0][2]

#     # Add a marker for the points with a different color
#     point_marker = Marker()
#     point_marker.header.frame_id = msg.header.frame_id
#     point_marker.header.stamp = rospy.Time.now()
#     point_marker.ns = "point_markers"
#     point_marker.id = 0
#     point_marker.type = Marker.POINTS
#     point_marker.action = Marker.ADD
#     point_marker.scale.x = 0.02
#     point_marker.scale.y = 0.02
#     point_marker.scale.z = 0.02
#     point_marker.color.a = 1.0
#     point_marker.pose.orientation.w = 1.0
    
#     for x, y, z in xz_points[1:]:
#         dz = z - prev_z
#         dx = x - prev_x

        
#         pt = Point()
#         pt.x = dx
#         pt.y = 0
#         pt.z = dz
#         point_marker.points.append(pt)

#         total = abs(dx) + abs(dz)
#         r = abs(dx) / total if total > 0 else 0.5
#         b = abs(dz) / total if total > 0 else 0.5

#         color = ColorRGBA()
#         color.r = r
#         color.g = 0.0
#         color.b = b
#         color.a = 1.0
#         point_marker.colors.append(color)

#         if min_step_height <= dz <= max_step_height and dx >= min_step_depth:
#             step_count += 1
#             prev_z = z
#             prev_x = x

#         rospy.loginfo(f"dx={dx:.4f} m, dz={dz:.4f} m z ={z:.4f} m x={x:.4f} m")

#     marker_pub.publish(point_marker)

    
# if __name__ == '__main__':
#     rospy.init_node('stair_detector_node')

#     rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_detection_callback)

#     marker_pub = rospy.Publisher('/stair_markers', Marker, queue_size=1)

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


# import rospy
# import math
# import numpy as np
# import open3d as o3d
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Bool
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point


# def stair_detection_callback(msg):

#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     xz_points = [(x, y, z) for x, y, z in points]
    

#     if len(xz_points) < 10:
#         stair_pub.publish(Bool(data=False))
#         return
    



#     # Convert to Open3D format
#     pc_array = np.array(xz_points, dtype=np.float32)
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(pc_array)

#     stair_count = 0

#     # Marker initialization
#     marker = Marker()
#     marker.header.frame_id = msg.header.frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "stairs"
#     marker.id = 0
#     marker.type = Marker.LINE_LIST
#     marker.action = Marker.ADD
#     marker.scale.x = 0.02
#     marker.scale.y = 0.02
#     marker.scale.z = 0.02
#     marker.color.a = 1.0
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0

#     # Fix orientation
#     marker.pose.orientation.w = 1.0
#     marker.pose.orientation.x = 0.0
#     marker.pose.orientation.y = 0.0
#     marker.pose.orientation.z = 0.0

#     # Plane segmentation loop
#     baseline_step_height = None
#     baseline_step_distance = None

#     while len(pcd.points) > 30:
#         plane_model, inliers = pcd.segment_plane(
#             distance_threshold=0.06,
#             ransac_n=3,
#             num_iterations=1000
#         )

#         [a, b, c, d] = plane_model

#         # Skip horizontal planes (ground or ceiling)
#         if abs(c) > 0.9:
#             pcd = pcd.select_by_index(inliers, invert=True)
#             continue

#         # Check if the plane is a potential riser
#         if 0.1 < abs(a) < 0.6 and abs(c) > 0.7:
#             inlier_cloud = pcd.select_by_index(inliers)
#             inlier_points = np.asarray(inlier_cloud.points)

#             # Calculate the average height (z) and distance (x) of the riser
#             avg_z = np.mean(inlier_points[:, 2])
#             avg_x = np.mean(inlier_points[:, 0])

#             print(f"avg_z: {avg_z:.3f} m, avg_x: {avg_x:.3f} m")

#             if baseline_step_height is None or baseline_step_distance is None:
#                 baseline_step_height = avg_z
#                 baseline_step_distance = avg_x
#                 stair_count += 1

#                 # Add marker for the first riser
#                 for pt in inlier_points:
#                     p1 = Point(pt[0], 0.0, pt[2])
#                     p2 = Point(pt[0], 0.0, pt[2] + 0.1)
#                     marker.points.append(p1)
#                     marker.points.append(p2)
#             else:
#                 height_diff = abs(avg_z - baseline_step_height)
#                 distance_diff = abs(avg_x - baseline_step_distance)

#                 if 0.05 <= height_diff <= 0.4 and 0.8 * baseline_step_distance <= distance_diff <= 1.2 * baseline_step_distance:
#                     stair_count += 1

#                     baseline_step_height = avg_z
#                     baseline_step_distance = avg_x

#                     # Add marker for the riser
#                     for pt in inlier_points:
#                         p1 = Point(pt[0], 0.0, pt[2])
#                         p2 = Point(pt[0], 0.0, pt[2] + 0.1)
#                         marker.points.append(p1)
#                         marker.points.append(p2)

#         # Remove the detected plane from point cloud
#         pcd = pcd.select_by_index(inliers, invert=True)

#     # Only publish marker if points exist
#     if len(marker.points) > 0:
#         marker_pub.publish(marker)
#     else:
#         rospy.logwarn("Empty marker not published.")

#     # Publish stair detection status
#     if stair_count >= 2:
#         # rospy.loginfo(f"STAIRS DETECTED: {stair_count} risers found.")
#         stair_pub.publish(Bool(data=True))
#     else:
#         # rospy.loginfo("No stairs detected in this frame.")
#         stair_pub.publish(Bool(data=False))


# if __name__ == '__main__':
#     rospy.init_node('stair_detector_node_ransac')

#     rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_detection_callback)

#     stair_pub = rospy.Publisher('/stairs_detected', Bool, queue_size=1)
#     marker_pub = rospy.Publisher('/stair_markers', Marker, queue_size=1)

#     # rospy.loginfo("Stair Detection Node Started (RANSAC-based, Safe Publishing)...")
#     rospy.spin()


# import rospy
# import numpy as np
# import open3d as o3d
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point

# def stair_detection_callback(msg):
#     # Convert PointCloud2 to numpy array
#     points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#     xz_points = [(x, y, z) for x, y, z in points]

#     if len(xz_points) < 30:
#         return

#     pc_array = np.array(xz_points, dtype=np.float32)
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(pc_array)

#     # Initialize visualization marker
#     marker = Marker()
#     marker.header.frame_id = msg.header.frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "stairs"
#     marker.id = 0
#     marker.type = Marker.LINE_LIST
#     marker.action = Marker.ADD
#     marker.scale.x = 0.05
#     marker.color.a = 1.0
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.pose.orientation.w = 1.0

#     risers = []

#     # Step 1: RANSAC Plane Segmentation Loop
#     while len(pcd.points) > 30:
#         plane_model, inliers = pcd.segment_plane(
#             distance_threshold=0.05,
#             ransac_n=3,
#             num_iterations=1000
#         )
#         [a, b, c, d] = plane_model

#         # Step 2: Ignore horizontal planes (ground/floor)
#         if abs(c) > 0.9:
#             pcd = pcd.select_by_index(inliers, invert=True)
#             continue

#         # Step 3: Candidate vertical plane (step riser)
#         if abs(a) > 0.2 and abs(c) > 0.7:
#             inlier_cloud = pcd.select_by_index(inliers)
#             inlier_points = np.asarray(inlier_cloud.points)

#             if inlier_points.shape[0] == 0:
#                 pcd = pcd.select_by_index(inliers, invert=True)
#                 continue

#             sorted_points = inlier_points[inlier_points[:, 0].argsort()]
#             center_x = np.median(sorted_points[:, 0])
#             center_z = np.min(sorted_points[:, 2])
#             height = np.max(sorted_points[:, 2]) - center_z

#             print(f"Riser Candidate: x={center_x:.2f}, z={center_z:.2f}, height={height:.2f}")

#             # Store riser candidate (x, z, height)
#             risers.append((center_x, center_z, height))

#         # Remove this plane and continue
#         pcd = pcd.select_by_index(inliers, invert=True)

#     # Step 4: Filter risers based on spacing (to avoid overlaps)
#     risers.sort(key=lambda r: r[0])  # sort by x
#     filtered_risers = []
#     last_x = None
#     min_spacing = 0.2

#     for x, z, h in risers:
#         if last_x is None or abs(x - last_x) > min_spacing:
#             filtered_risers.append((x, z, h))
#             last_x = x
#         # print(f"Filtered Riser: x={x:.2f}, z={z:.2f}, height={h:.2f}")
    

#     # Step 5: Generate 1 vertical line marker per riser
#     for x, z, h in filtered_risers:
#         p1 = Point(x, 0.0, z)
#         p2 = Point(x, 0.0, z+0.19)
#         marker.points.append(p1)
#         marker.points.append(p2)

#     if marker.points:
#         marker_pub.publish(marker)


# if __name__ == '__main__':
#     rospy.init_node('stair_detector_node_ransac')
#     rospy.Subscriber('/livox/xz_plane', PointCloud2, stair_detection_callback)
#     marker_pub = rospy.Publisher('/stair_markers', Marker, queue_size=1)

#     rospy.loginfo("Stair Detection Node Started (Improved RANSAC)...")
#     rospy.spin()
