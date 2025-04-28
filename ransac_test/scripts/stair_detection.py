#!/usr/bin/env python3

import rospy
import math
import pandas as pd
import random
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from livox_ros_driver2.msg import CustomMsg


class RANSAC:
    def __init__(self, point_cloud, max_iterations=500, distance_threshold=0.2):
        self.point_cloud = point_cloud
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold

    def run(self):
        best_inliers = set()
        for _ in range(self.max_iterations):
            sample = self.point_cloud.sample(n=3)
            x1, y1, z1 = sample.iloc[0]
            x2, y2, z2 = sample.iloc[1]
            x3, y3, z3 = sample.iloc[2]

            # Plane coefficients
            a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
            b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
            c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
            d = -(a * x1 + b * y1 + c * z1)
            norm = math.sqrt(a**2 + b**2 + c**2)
            if norm == 0:
                continue  

            inliers = set()
            for idx, (x, y, z) in self.point_cloud.iterrows():
                distance = abs(a*x + b*y + c*z + d) / norm
                if distance <= self.distance_threshold:
                    inliers.add(idx)

            if len(inliers) > len(best_inliers):
                best_inliers = inliers

        inlier_df = self.point_cloud.loc[list(best_inliers)].reset_index(drop=True)
        return inlier_df


# Global publisher
plane_pub = None

def livox_callback(msg: CustomMsg):
    points = []
    for pt in msg.points:
        x, y, z = pt.x, pt.y, pt.z
        if not any(map(math.isnan, (x, y, z))):
            points.append((x, y, z))

    if len(points) < 10:
        rospy.logwarn("Not enough points to run RANSAC.")
        return 

    df = pd.DataFrame(points, columns=["X", "Y", "Z"])

    rospy.loginfo(f"Points: {df}")

    # Run RANSAC
    ransac = RANSAC(df)
    plane_df = ransac.run()

    # Publish inlier plane points
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id  
    cloud = pc2.create_cloud_xyz32(header, plane_df[["X", "Y", "Z"]].values.tolist())
    plane_pub.publish(cloud)
    rospy.loginfo(f"Published {len(plane_df)} RANSAC inliers as plane")


if __name__ == '__main__':
    rospy.init_node('livox_ransac_plane_node')

    plane_pub = rospy.Publisher('/ransac_plane', PointCloud2, queue_size=1)

    rospy.Subscriber('/livox/lidar', CustomMsg, livox_callback)

    rospy.loginfo("Listening to /livox/lidar and running RANSAC plane segmentation...")
    rospy.spin()
