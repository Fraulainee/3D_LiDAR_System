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

# -------------------- state / defaults --------------------
last_pub_time = rospy.Time(0)

# use separate buffers per band (rolling median)
BUFFER_SIZE = 20
height_buf_low  = deque(maxlen=BUFFER_SIZE)
height_buf_high = deque(maxlen=BUFFER_SIZE)

MISS_THRESHOLD = 3
consecutive_miss_counter = 0

# Defaults when no detection
DEF_DISTANCE = 1000.0
DEF_HEIGHT   = 0.0
DEF_ANGLE    = 0.0

# -------------------- helpers --------------------
def generate_text_marker(height, x, y, z, frame_id, marker_id, label):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = "log_marker_text"
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z + 0.10
    m.pose.orientation.w = 1.0
    m.scale.z = 0.10          # text size (m)
    m.color.a = 1.0
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    prefix = (label + ": ") if label else ""
    m.text = "{}{:.2f} m, {:.2f} m away".format(prefix, height, x)
    return m

def line_strip_marker(cx, cy, base_z, top_z, frame_id, marker_id, rgba):
    """Vertical line (2-point LINE_STRIP) placed at cluster centroid (cx,cy)."""
    lm = Marker()
    lm.header.frame_id = frame_id
    lm.header.stamp = rospy.Time.now()
    lm.ns = "log_marker"
    lm.id = marker_id
    lm.type = Marker.LINE_STRIP
    lm.action = Marker.ADD
    lm.scale.x = 0.03  # line width
    a, r, g, b = rgba
    lm.color.a = a; lm.color.r = r; lm.color.g = g; lm.color.b = b
    lm.pose.orientation.w = 1.0
    lm.points = [Point(cx, cy, base_z),
                 Point(cx, cy, top_z)]
    return lm

def delete_marker_pair(frame_id, marker_id):
    """Delete the line (id) and its paired text (id+10)."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = "log_marker"
    m.id = marker_id
    m.action = Marker.DELETE
    marker_pub.publish(m)

    t = Marker()
    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.ns = "log_marker_text"
    t.id = marker_id + 10
    t.action = Marker.DELETE
    marker_pub.publish(t)

def clear_both_markers(frame_id):
    delete_marker_pair(frame_id, 0)
    delete_marker_pair(frame_id, 1)

def band_stats(band_xyz):
    """
    band_xyz: (N,3) array of [x,y,z]
    Returns (cx, cy, base_z, top_z, height, distance)
    """
    cx = float(np.mean(band_xyz[:, 0]))
    cy = float(np.mean(band_xyz[:, 1]))
    base_z = float(np.min(band_xyz[:, 2]))
    top_z  = float(np.max(band_xyz[:, 2]))
    height = top_z - base_z + 0.10       # keep your +0.1 bias (tune later if needed)
    distance = math.hypot(cx, base_z) - 0.025
    return cx, cy, base_z, top_z, height, distance

def kmeans2_xz(points_xz, max_iters=20):
    """
    Minimal K-Means for k=2 over (x,z). Returns labels(0/1), centroids(2,2).
    """
    P = points_xz.astype(np.float32)
    if len(P) < 2:
        return np.zeros((len(P),), dtype=np.int32), np.array([[0,0],[0,0]], dtype=np.float32)

    # Init by x-quantiles for stability
    order = np.argsort(P[:, 0])
    q = max(1, len(P)//3)
    c0 = P[order[:q]].mean(axis=0)
    c1 = P[order[-q:]].mean(axis=0)
    C = np.stack([c0, c1], axis=0)

    for _ in range(max_iters):
        d0 = np.sum((P - C[0])**2, axis=1)
        d1 = np.sum((P - C[1])**2, axis=1)
        labels = (d1 < d0).astype(np.int32)

        C_new = np.zeros_like(C)
        for k in (0, 1):
            mask = (labels == k)
            C_new[k] = P[mask].mean(axis=0) if np.any(mask) else P[np.random.randint(len(P))]

        if np.allclose(C, C_new, atol=1e-4):
            break
        C = C_new

    return labels, C

def smooth_and_angle(label, height_raw, distance):
    """
    Update the correct buffer (LOW/HIGH), return (height_smooth, angle_deg).
    Uses rolling median for stability; per-band so values won't be identical.
    """
    if label == "LOW":
        height_buf_low.append(height_raw)
        h = float(np.median(height_buf_low))
    else:
        height_buf_high.append(height_raw)
        h = float(np.median(height_buf_high))

    angle_deg = math.degrees(math.atan2(h, max(distance, 1e-6)))
    return h, angle_deg

# -------------------- main callback --------------------
def log_detection_callback(msg):
    global last_pub_time, consecutive_miss_counter

    # live-tunable params
    min_points_per_band = int(rospy.get_param('~min_points_per_band', 20))
    min_centroid_sep_x  = float(rospy.get_param('~min_centroid_sep_x', 0.06))  # m
    min_height_diff     = float(rospy.get_param('~min_height_diff', 0.06))     # m
    z_window            = float(rospy.get_param('~z_window', 0.15))            # +/- around median

    # Read all points (x,y,z); keep only x > 0 (front)
    pts_iter = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
    xyz_points = [(x, y, z) for x, y, z in pts_iter if x > 0.0]

    if len(xyz_points) < 5:
        consecutive_miss_counter += 1
        if consecutive_miss_counter >= MISS_THRESHOLD:
            log_pub.publish(Bool(data=False))
            clear_both_markers(msg.header.frame_id)
            # publish default info every ~2s
            if (rospy.Time.now() - last_pub_time).to_sec() >= 2.0:
                for pub in (stair_info_pub_high, stair_info_pub_low):
                    si = StairInfo(); si.distance, si.height, si.angle = DEF_DISTANCE, DEF_HEIGHT, DEF_ANGLE
                    pub.publish(si)
                last_pub_time = rospy.Time.now()
        return
    else:
        consecutive_miss_counter = 0

    xyz_np = np.array(xyz_points, dtype=np.float32)

    # Median-based z window (prefilter)
    z_med = np.median(xyz_np[:, 2])
    mask = (xyz_np[:, 2] >= z_med - z_window) & (xyz_np[:, 2] <= z_med + z_window)
    log_candidates = xyz_np[mask]
    if len(log_candidates) < 10:
        log_pub.publish(Bool(data=False))
        clear_both_markers(msg.header.frame_id)
        return

    # -------- K-Means split (k=2) by (x,z) --------
    labels, _ = kmeans2_xz(log_candidates[:, [0, 2]])  # x,z
    band0 = log_candidates[labels == 0]   # full xyz
    band1 = log_candidates[labels == 1]

    valid0 = len(band0) >= min_points_per_band
    valid1 = len(band1) >= min_points_per_band
    if not (valid0 and valid1):
        # need two obstacles; otherwise clear & exit
        log_pub.publish(Bool(data=False))
        clear_both_markers(msg.header.frame_id)
        # also publish defaults so downstream knows "no two obstacles"
        for pub in (stair_info_pub_high, stair_info_pub_low):
            si = StairInfo(); si.distance, si.height, si.angle = DEF_DISTANCE, DEF_HEIGHT, DEF_ANGLE
            pub.publish(si)
        return

    # Compute stats & check distinctness
    s0 = band_stats(band0)  # (cx,cy,baseZ,topZ,h,dist)
    s1 = band_stats(band1)
    h0, h1 = s0[4], s1[4]
    cx0, cx1 = s0[0], s1[0]
    height_diff = abs(h1 - h0)
    centroid_sep_x = abs(cx1 - cx0)

    if not ((centroid_sep_x >= min_centroid_sep_x) or (height_diff >= min_height_diff)):
        # not distinct -> clear and exit
        log_pub.publish(Bool(data=False))
        clear_both_markers(msg.header.frame_id)
        for pub in (stair_info_pub_high, stair_info_pub_low):
            si = StairInfo(); si.distance, si.height, si.angle = DEF_DISTANCE, DEF_HEIGHT, DEF_ANGLE
            pub.publish(si)
        return

    # Label LOW/HIGH by height
    if h0 <= h1:
        low_stats, high_stats = s0, s1
    else:
        low_stats, high_stats = s1, s0

    # Colors: LOW green-ish, HIGH magenta
    color_low  = (1.0, 0.1, 0.9, 0.1)   # a,r,g,b
    color_high = (1.0, 1.0, 0.0, 1.0)

    # -------- Publish LOW (with per-band smoothing) --------
    cx, cy, base_z, top_z, height_raw, dist = low_stats
    if height_raw >= 0.015:
        h_smooth, angle_deg = smooth_and_angle("LOW", height_raw, dist)
        marker_pub.publish(line_strip_marker(cx, cy, base_z, top_z, msg.header.frame_id, 0, color_low))
        marker_pub.publish(generate_text_marker(h_smooth, cx, cy, top_z, msg.header.frame_id, 10, "LOW"))
        si = StairInfo(); si.distance = dist; si.height = h_smooth; si.angle = angle_deg
        stair_info_pub_low.publish(si)
    else:
        delete_marker_pair(msg.header.frame_id, 0)

    # -------- Publish HIGH (with per-band smoothing) --------
    cx, cy, base_z, top_z, height_raw, dist = high_stats
    if height_raw >= 0.015:
        h_smooth, angle_deg = smooth_and_angle("HIGH", height_raw, dist)
        marker_pub.publish(line_strip_marker(cx, cy, base_z, top_z, msg.header.frame_id, 1, color_high))
        marker_pub.publish(generate_text_marker(h_smooth, cx, cy, top_z, msg.header.frame_id, 11, "HIGH"))
        si = StairInfo(); si.distance = dist; si.height = h_smooth; si.angle = angle_deg
        stair_info_pub_high.publish(si)
    else:
        delete_marker_pair(msg.header.frame_id, 1)

    log_pub.publish(Bool(data=True))

# -------------------- node setup --------------------
if __name__ == '__main__':
    rospy.init_node('log_detector_node')

    rospy.Subscriber('/livox/xz_plane', PointCloud2, log_detection_callback)

    log_pub    = rospy.Publisher('/log_detected', Bool, queue_size=1)
    marker_pub = rospy.Publisher('/log_marker', Marker, queue_size=8)

    # Only two info topics now:
    stair_info_pub_high = rospy.Publisher('/log_info_high', StairInfo, queue_size=1)
    stair_info_pub_low  = rospy.Publisher('/log_info_low',  StairInfo, queue_size=1)

    rospy.loginfo("Two-Obstacle detector running (per-band height smoothing). "
                  "Tune with ~min_points_per_band, ~min_centroid_sep_x, ~min_height_diff, ~z_window")
    rospy.spin()
