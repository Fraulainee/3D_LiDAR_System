#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from stair_detection.msg import StairInfo
from collections import deque

last_pub_time = rospy.Time(0)
BUFFER_SIZE = 20
height_buf_left  = deque(maxlen=BUFFER_SIZE)
height_buf_right = deque(maxlen=BUFFER_SIZE)
MISS_THRESHOLD = 3
consecutive_miss_counter = 0

DEF_DISTANCE = 1000.0
DEF_HEIGHT   = 0.0
DEF_ANGLE    = 0.0

def generate_text_marker(height, x, y, z, frame_id, marker_id, label):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = "log_marker_text"
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z + 0.10
    m.pose.orientation.w = 1.0
    m.scale.z = 0.10
    m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0
    m.text = "{}: {:.2f} m, {:.2f} m away".format(label, height, x)
    return m

def line_strip_marker(cx, cy, base_z, top_z, frame_id, marker_id, rgba):
    lm = Marker()
    lm.header.frame_id = frame_id
    lm.header.stamp = rospy.Time.now()
    lm.ns = "log_marker"
    lm.id = marker_id
    lm.type = Marker.LINE_STRIP
    lm.action = Marker.ADD
    lm.scale.x = 0.03
    a, r, g, b = rgba
    lm.color.a = a; lm.color.r = r; lm.color.g = g; lm.color.b = b
    lm.pose.orientation.w = 1.0
    lm.points = [Point(cx, cy, base_z), Point(cx, cy, top_z)]
    return lm

def delete_marker_pair(frame_id, marker_id):
    m = Marker(); m.header.frame_id = frame_id; m.header.stamp = rospy.Time.now()
    m.ns = "log_marker"; m.id = marker_id; m.action = Marker.DELETE; marker_pub.publish(m)
    t = Marker(); t.header.frame_id = frame_id; t.header.stamp = rospy.Time.now()
    t.ns = "log_marker_text"; t.id = marker_id + 10; t.action = Marker.DELETE; marker_pub.publish(t)

def clear_both_markers(frame_id):
    delete_marker_pair(frame_id, 0); delete_marker_pair(frame_id, 1)

def band_stats(band_xyz):
    cx = float(np.mean(band_xyz[:, 0]))  # x
    cy = float(np.mean(band_xyz[:, 1]))  # y
    base_z = float(np.min(band_xyz[:, 2]))
    top_z  = float(np.max(band_xyz[:, 2]))
    height = top_z - base_z + 0.10
    distance = math.hypot(cx, base_z) - 0.025
    return cx, cy, base_z, top_z, height, distance

def kmeans2_2d(points_2d, max_iters=20):
    P = points_2d.astype(np.float32)
    if len(P) < 2:
        return np.zeros((len(P),), dtype=np.int32), np.array([[0,0],[0,0]], dtype=np.float32)
    order = np.argsort(P[:, 0])  # init on first axis (here: Y)
    q = max(1, len(P)//3)
    c0 = P[order[:q]].mean(axis=0); c1 = P[order[-q:]].mean(axis=0)
    C = np.stack([c0, c1], axis=0)
    for _ in range(max_iters):
        d0 = np.sum((P - C[0])**2, axis=1); d1 = np.sum((P - C[1])**2, axis=1)
        labels = (d1 < d0).astype(np.int32)
        C_new = np.zeros_like(C)
        for k in (0,1):
            m = (labels==k)
            C_new[k] = P[m].mean(axis=0) if np.any(m) else P[np.random.randint(len(P))]
        if np.allclose(C, C_new, atol=1e-4): break
        C = C_new
    return labels, C

def smooth_and_angle(side_label, height_raw, distance):
    if side_label == "LEFT":
        height_buf_left.append(height_raw); h = float(np.median(height_buf_left))
    else:
        height_buf_right.append(height_raw); h = float(np.median(height_buf_right))
    angle_deg = math.degrees(math.atan2(h, max(distance, 1e-6)))
    return h, angle_deg

def log_detection_callback(msg):
    global last_pub_time, consecutive_miss_counter

    min_points_per_band = int(rospy.get_param('~min_points_per_band', 12))
    min_centroid_sep_y  = float(rospy.get_param('~min_centroid_sep_y', 0.05))  # << NEW: ΔY gate
    min_height_diff     = float(rospy.get_param('~min_height_diff', 0.03))
    z_window            = float(rospy.get_param('~z_window', 0.20))

    pts_iter = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
    xyz_points = [(x, y, z) for x, y, z in pts_iter if x > 0.0]

    if len(xyz_points) < 5:
        consecutive_miss_counter += 1
        if consecutive_miss_counter >= MISS_THRESHOLD:
            log_pub.publish(Bool(data=False)); clear_both_markers(msg.header.frame_id)
            if (rospy.Time.now() - last_pub_time).to_sec() >= 2.0:
                for pub in (stair_info_pub_left, stair_info_pub_right):
                    si = StairInfo(); si.distance, si.height, si.angle = DEF_DISTANCE, DEF_HEIGHT, DEF_ANGLE
                    pub.publish(si); last_pub_time = rospy.Time.now()
        return
    else:
        consecutive_miss_counter = 0

    xyz_np = np.array(xyz_points, dtype=np.float32)

    z_med = np.median(xyz_np[:, 2])
    mask = (xyz_np[:, 2] >= z_med - z_window) & (xyz_np[:, 2] <= z_med + z_window)
    log_candidates = xyz_np[mask]
    if len(log_candidates) < 10:
        log_pub.publish(Bool(data=False)); clear_both_markers(msg.header.frame_id); return

    # ---- Split by (Y,Z) so left/right separation works ----
    labels, _ = kmeans2_2d(log_candidates[:, [1, 2]])  # [Y, Z]
    band0 = log_candidates[labels == 0]
    band1 = log_candidates[labels == 1]

    if len(band0) < min_points_per_band or len(band1) < min_points_per_band:
        log_pub.publish(Bool(data=False)); clear_both_markers(msg.header.frame_id)
        for pub in (stair_info_pub_left, stair_info_pub_right):
            si = StairInfo(); si.distance, si.height, si.angle = DEF_DISTANCE, DEF_HEIGHT, DEF_ANGLE
            pub.publish(si)
        return

    s0 = band_stats(band0); s1 = band_stats(band1)
    h0, h1 = s0[4], s1[4]
    cy0, cy1 = s0[1], s1[1]
    height_diff = abs(h1 - h0)
    centroid_sep_y = abs(cy1 - cy0)

    # distinct if separated laterally (ΔY) OR heights differ
    if not ((centroid_sep_y >= min_centroid_sep_y) or (height_diff >= min_height_diff)):
        log_pub.publish(Bool(data=False)); clear_both_markers(msg.header.frame_id)
        for pub in (stair_info_pub_left, stair_info_pub_right):
            si = StairInfo(); si.distance, si.height, si.angle = DEF_DISTANCE, DEF_HEIGHT, DEF_ANGLE
            pub.publish(si)
        return

    # Map to sides by Y sign: more negative Y = LEFT, more positive Y = RIGHT
    if cy0 <= cy1:
        left_stats, right_stats = s0, s1
    else:
        left_stats, right_stats = s1, s0

    color_left  = (1.0, 0.1, 0.9, 0.1)
    color_right = (1.0, 1.0, 0.0, 1.0)

    # LEFT
    cx, cy, base_z, top_z, height_raw, dist = left_stats
    if height_raw >= 0.005:  # allow smaller obstacles
        h_smooth, angle_deg = smooth_and_angle("LEFT", height_raw, dist)
        marker_pub.publish(line_strip_marker(cx, cy, base_z, top_z, msg.header.frame_id, 0, color_left))
        marker_pub.publish(generate_text_marker(h_smooth, cx, cy, top_z, msg.header.frame_id, 10, "LEFT"))
        si = StairInfo(); si.distance = dist; si.height = h_smooth; si.angle = angle_deg
        stair_info_pub_left.publish(si)
    else:
        delete_marker_pair(msg.header.frame_id, 0)

    # RIGHT
    cx, cy, base_z, top_z, height_raw, dist = right_stats
    if height_raw >= 0.005:
        h_smooth, angle_deg = smooth_and_angle("RIGHT", height_raw, dist)
        marker_pub.publish(line_strip_marker(cx, cy, base_z, top_z, msg.header.frame_id, 1, color_right))
        marker_pub.publish(generate_text_marker(h_smooth, cx, cy, top_z, msg.header.frame_id, 11, "RIGHT"))
        si = StairInfo(); si.distance = dist; si.height = h_smooth; si.angle = angle_deg
        stair_info_pub_right.publish(si)
    else:
        delete_marker_pair(msg.header.frame_id, 1)

    log_pub.publish(Bool(data=True))

if __name__ == '__main__':
    rospy.init_node('log_detector_node')

    rospy.Subscriber('/livox/xz_plane', PointCloud2, log_detection_callback)

    log_pub    = rospy.Publisher('/log_detected', Bool, queue_size=1)
    marker_pub = rospy.Publisher('/log_marker', Marker, queue_size=8)

    stair_info_pub_left  = rospy.Publisher('/log_info_left',  StairInfo, queue_size=1)
    stair_info_pub_right = rospy.Publisher('/log_info_right', StairInfo, queue_size=1)

    rospy.loginfo("Two-Obstacle detector (split on Y,Z; LEFT=neg Y, RIGHT=pos Y)")
    rospy.spin()
