#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

def obstacle_detect(msg):
    min_x = 0.10
    max_x = 2.0
    min_z = 0.05
    cluster_threshold = 0.15  # meters between obstacles

    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    # Only get points in valid region
    obstacle_points = [(x, z) for x, y, z in points if (min_x < x < max_x and z > min_z)]
    if not obstacle_points:
        return

    # Sort points by X
    obstacle_points.sort(key=lambda p: p[0])

    # 1D clustering by X
    clusters = []
    current_cluster = []
    prev_x = None
    for x, z in obstacle_points:
        if prev_x is None or abs(x - prev_x) < cluster_threshold:
            current_cluster.append((x, z))
        else:
            if current_cluster:
                clusters.append(current_cluster)
            current_cluster = [(x, z)]
        prev_x = x
    if current_cluster:
        clusters.append(current_cluster)

    # Remove previous markers (optional but recommended for MarkerArray)
    marker_array = MarkerArray()
    for i, cluster in enumerate(clusters):
        xs, zs = zip(*cluster)
        mean_x = sum(xs) / len(xs)
        mean_z = sum(zs) / len(zs)
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacle"
        marker.id = i
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = mean_x
        marker.pose.position.y = 0
        marker.pose.position.z = mean_z / 2
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = max(mean_z, 0.05)
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.b = 0.2
        marker.lifetime = rospy.Duration(0.2)  
        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)
    rospy.loginfo(f"Detected {len(clusters)} obstacles.")

if __name__ == '__main__':
    rospy.init_node('obstacle_detector_node')
    global marker_pub
    from visualization_msgs.msg import MarkerArray
    marker_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=1)
    rospy.Subscriber('/livox/xz_plane', PointCloud2, obstacle_detect)
    rospy.loginfo("Obstacle Detection Node Started...")
    rospy.spin()
