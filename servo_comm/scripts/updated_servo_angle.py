#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time
from stair_detection.msg import ServoLeft
from stair_detection.msg import ServoRight

class DualAngleSerialSender:
    def __init__(self):
        # ---- Params
        self.port = rospy.get_param('~serial_port', '/dev/ttyACM2')
        self.baud = int(rospy.get_param('~baud', 9600))
        self.startup_wait = float(rospy.get_param('~startup_wait', 2.0))   # Arduino reset delay
        self.threshold_m = float(rospy.get_param('~threshold_m', 0.30))    # (unused here)
        self.min_interval_s = float(rospy.get_param('~min_interval_s', 0.05))  # overall rate limit
        self.angle_min = int(rospy.get_param('~angle_min', 0))
        self.angle_max = int(rospy.get_param('~angle_max', 180))
        self.init_low_angle  = int(rospy.get_param('~init_low_angle',  90))
        self.init_high_angle = int(rospy.get_param('~init_high_angle', 90))

        # ---- State
        self.low_angle  = self._clamp(self.init_low_angle)
        self.high_angle = self._clamp(self.init_high_angle)
        self.low_dist   = float('inf')
        self.high_dist  = float('inf')
        self.last_send_time = rospy.Time(0)

        # ---- Serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(self.startup_wait)
            rospy.loginfo(f"[SERIAL] Opened {self.port} @ {self.baud}")
        except Exception as e:
            rospy.logerr(f"[SERIAL] Could not open port {self.port}: {e}")
            raise

        # ---- Subs
        rospy.Subscriber('/log_info_low',  ServoLeft,  self.cb_low,  queue_size=10)
        rospy.Subscriber('/log_info_high', ServoRight, self.cb_high, queue_size=10)

    # ---------- callbacks ----------
    def cb_low(self, msg: ServoLeft):
        if hasattr(msg, 'angle') and msg.angle is not None:
            self.low_angle = self._clamp(msg.angle)

    def cb_high(self, msg: ServoRight):
        if hasattr(msg, 'angle') and msg.angle is not None:
            self.high_angle = self._clamp(msg.angle)

    # ---------- helper ----------
    def _clamp(self, v):
        try:
            a = int(round(float(v)))
        except Exception:
            a = 0
        if a < self.angle_min: a = self.angle_min
        if a > self.angle_max: a = self.angle_max
        return a


if __name__ == '__main__':
    rospy.init_node('dual_angle_serial_sender')
    node = DualAngleSerialSender()

    # simple loop that sends "low high\n" at the requested interval
    rate_hz = 1.0 / node.min_interval_s if node.min_interval_s > 0.0 else 50.0
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        # Only send if both angles are known
        if node.low_angle is not None and node.high_angle is not None:
            # BYPASS: skip sending when both angles are zero
            if node.low_angle == 0 and node.high_angle == 0:
                rate.sleep()
                continue

            now = rospy.Time.now()
            if (now - node.last_send_time).to_sec() >= node.min_interval_s:
                line = f"{int(node.low_angle)} {int(node.high_angle)}\n"
                try:
                    node.ser.write(line.encode('ascii'))
                    node.last_send_time = now
                    rospy.loginfo(f"[SEND] {line.strip()}")
                except Exception as e:
                    rospy.logerr(f"[SERIAL] Write failed: {e}")
        rate.sleep()
