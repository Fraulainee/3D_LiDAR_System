#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import time
from stair_detection.msg import StairInfo  # fields: distance (m), angle (deg)

class DualAngleSerialSender:
    def __init__(self):
        # ---- Params
        self.port = rospy.get_param('~serial_port', '/dev/ttyACM2')
        self.baud = int(rospy.get_param('~baud', 9600))
        self.startup_wait = float(rospy.get_param('~startup_wait', 2.0))   # Arduino reset delay
        self.threshold_m = float(rospy.get_param('~threshold_m', 0.30))    # distance gate
        self.min_interval_s = float(rospy.get_param('~min_interval_s', 0.05))  # overall rate limit
        self.angle_min = int(rospy.get_param('~angle_min', 0))
        self.angle_max = int(rospy.get_param('~angle_max', 180))
        self.init_low_angle  = int(rospy.get_param('~init_low_angle',  90))
        self.init_high_angle = int(rospy.get_param('~init_high_angle', 90))

        # When to send:
        #   'always' -> send pair on any update
        #   'any'    -> send if LOW or HIGH distance < threshold_m (default)
        #   'both'   -> send only if both distances < threshold_m
        self.send_mode = rospy.get_param('~send_mode', 'any').lower()
        if self.send_mode not in ('always', 'any', 'both'):
            rospy.logwarn("~send_mode should be 'always'|'any'|'both'; using 'any'")
            self.send_mode = 'any'

        # ---- State
        self.low_angle  = None
        self.high_angle = None
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
        rospy.Subscriber('/log_info_low',  StairInfo, self.cb_low,  queue_size=10)
        rospy.Subscriber('/log_info_high', StairInfo, self.cb_high, queue_size=10)

        # Initialize with defaults so first send has two numbers
        self.low_angle  = self._format_angle(self.init_low_angle)
        self.high_angle = self._format_angle(self.init_high_angle)

    # ---------- callbacks ----------
    def cb_low(self, msg: StairInfo):
        self.low_dist  = msg.distance if msg.distance is not None else float('inf')
        self.low_angle = self._format_angle(msg.angle)
        self._maybe_send(trigger='LOW')

    def cb_high(self, msg: StairInfo):
        self.high_dist  = msg.distance if msg.distance is not None else float('inf')
        self.high_angle = self._format_angle(msg.angle)
        self._maybe_send(trigger='HIGH')

    # ---------- helpers ----------
    def _format_angle(self, angle_deg):
        try:
            a = int(round(float(angle_deg)))
        except Exception:
            a = 0
        if a < self.angle_min: a = self.angle_min
        if a > self.angle_max: a = self.angle_max
        return a

    def _gate(self):
        """Return True if we should send based on send_mode/thresholds."""
        if self.send_mode == 'always':
            return True
        low_ok  = (self.low_dist  < self.threshold_m)
        high_ok = (self.high_dist < self.threshold_m)
        if self.send_mode == 'both':
            return low_ok and high_ok
        # 'any'
        return low_ok or high_ok

    def _maybe_send(self, trigger='?'):
        # must have both angles known
        if self.low_angle is None or self.high_angle is None:
            return

        # rate limit
        now = rospy.Time.now()
        if (now - self.last_send_time).to_sec() < self.min_interval_s:
            return

        # distance gating
        if not self._gate():
            return

        line = f"{self.low_angle} {self.high_angle}\n"
        try:
            self.ser.write(line.encode('ascii'))
            self.last_send_time = now
            rospy.loginfo(f"[SEND {trigger}] low={self.low_angle} high={self.high_angle} "
                          f"(dl={self.low_dist:.2f} dh={self.high_dist:.2f}) -> '{line.strip()}'")
        except Exception as e:
            rospy.logerr(f"[SERIAL] Write failed: {e}")

if __name__ == '__main__':
    rospy.init_node('dual_angle_serial_sender')
    DualAngleSerialSender()
    rospy.spin()
