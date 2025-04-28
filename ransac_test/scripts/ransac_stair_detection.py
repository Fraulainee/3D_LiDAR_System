# #!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
# import std_msgs.msg
# import numpy as np
# import pandas as pd
# import math
# import random

# # Import Livox custom message
# from livox_ros_driver2.msg import CustomMsg


# class RANSAC:
#     def __init__(self, point_cloud, max_iterations, distance_ratio_threshold):
#         self.point_cloud = point_cloud
#         self.max_iterations = max_iterations
#         self.distance_ratio_threshold = distance_ratio_threshold

#     def run(self):
#         inliers, _ = self._ransac_algorithm(self.max_iterations, self.distance_ratio_threshold)
#         return inliers

#     def _ransac_algorithm(self, max_iterations, distance_ratio_threshold):
#         inliers_result = set()
#         while max_iterations:
#             max_iterations -= 1
#             inliers = []
#             while len(inliers) < 3:
#                 random_index = random.randint(0, len(self.point_cloud.X)-1)
#                 inliers.append(random_index)

#             x1, y1, z1 = self.point_cloud.loc[inliers[0]]
#             x2, y2, z2 = self.point_cloud.loc[inliers[1]]
#             x3, y3, z3 = self.point_cloud.loc[inliers[2]]

#             a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
#             b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
#             c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
#             d = -(a*x1 + b*y1 + c*z1)
#             plane_length = max(0.1, math.sqrt(a*a + b*b + c*c))

#             for idx, point in self.point_cloud.iterrows():
#                 if idx in inliers:
#                     continue
#                 x, y, z = point
#                 distance = abs(a*x + b*y + c*z + d) / plane_length
#                 if distance <= distance_ratio_threshold:
#                     inliers.append(idx)

#             if len(inliers) > len(inliers_result):
#                 inliers_result = set(inliers)

#         inlier_points = self.point_cloud.loc[list(inliers_result)].reset_index(drop=True)
#         outlier_points = self.point_cloud.drop(index=inliers_result).reset_index(drop=True)
#         return inlier_points, outlier_points


# # ROS Publisher
# plane_pub = None


# def livox_callback(msg: CustomMsg):
#     # Convert Livox CustomMsg to XYZ points
#     points = []
#     for pt in msg.points:
#         x, y, z = pt.x, pt.y, pt.z
#         if not any(map(math.isnan, (x, y, z))):
#             points.append((x, y, z))
#             rospy.loginfo(f"{points}")

#     if len(points) < 10:
#         rospy.logwarn("Not enough valid points to run RANSAC.")
#         return

#     # Convert to DataFrame
#     df = pd.DataFrame(points, columns=["X", "Y", "Z"])

#     # Run RANSAC
#     ransac = RANSAC(df, max_iterations=50, distance_ratio_threshold=0.2)
#     inliers = ransac.run()

#     # Convert inliers to PointCloud2 message
#     header = std_msgs.msg.Header()
#     header.stamp = rospy.Time.now()
#     header.frame_id = msg.header.frame_id  # Use same frame as input
#     cloud_msg = pc2.create_cloud_xyz32(header, inliers[["X", "Y", "Z"]].values.tolist())

#     # Publish
#     plane_pub.publish(cloud_msg)


# if __name__ == '__main__':
#     rospy.init_node('ransac_plane_segmenter')

#     plane_pub = rospy.Publisher('/ransac_plane', PointCloud2, queue_size=1)

#     rospy.Subscriber('/livox/lidar', CustomMsg, livox_callback)

#     rospy.loginfo("RANSAC plane segmentation running on Livox Mid-360 data...")
#     rospy.spin()


# #!/usr/bin/env python3

# import rospy
# import math
# from livox_ros_driver2.msg import CustomMsg


# def livox_callback(msg):
#     distances = []
#     for pt in msg.points:
#         x, y, z = pt.x, pt.y, pt.z
#         distance = math.sqrt(x**2 + y**2 + z**2)
#         distances.append(distance)

#     total_points = len(distances)
#     if total_points == 0:
#         rospy.logwarn("No points received.")
#         return

#     # avg_distance = sum(distances) / total_points

#     rospy.loginfo(f"Frame received: {total_points} points")
#     # rospy.loginfo(f"Average distance: {avg_distance:.2f} m")
#     for i, d in enumerate(distances):
#         rospy.loginfo(f"  Point {i}: distance = {d:.2f} m")


# if __name__ == '__main__':
#     rospy.init_node('livox_distance_node')
#     rospy.Subscriber('/livox/lidar', CustomMsg, livox_callback)
#     rospy.loginfo("Subscribed to /livox/lidar. Calculating distances...")
#     rospy.spin()

#!/usr/bin/env python3

import rospy
import math
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg


def livox_callback(msg):
    threshold = 0.5  

    filtered_points = []

    for pt in msg.points:
        x, y, z = pt.x, pt.y, pt.z
        intensity = float(pt.reflectivity)

        # rospy.loginfo(f"y={pt}")

        if abs(y) < threshold and x > 0:
            # filtered_points.append([x, y, z, intensity])
            filtered_points.append([x, y, z])

    # total_points = len(filtered_points)
    # rospy.loginfo(f"Filtered XZ Front Points: {total_points}")

    # Create PointCloud2 msg
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id  # Usually "livox_frame" or "map"

    # fields = [
    #     PointField('x', 0, PointField.FLOAT32, 1),
    #     PointField('y', 4, PointField.FLOAT32, 1),
    #     PointField('z', 8, PointField.FLOAT32, 1),
    #     PointField('intensity', 12, PointField.FLOAT32, 1),
    # ]

    cloud_msg = pc2.create_cloud_xyz32(header, filtered_points)

    pub.publish(cloud_msg)


if __name__ == '__main__':
    rospy.init_node('livox_xz_plane_node')

    pub = rospy.Publisher('/livox/xz_plane', PointCloud2, queue_size=1)

    rospy.Subscriber('/livox/lidar', CustomMsg, livox_callback)
    rospy.loginfo("Listening to /livox/lidar - Filtering XZ Plane Front Points...")

    rospy.spin()



# #!/usr/bin/env python3

# import rospy
# import math
# import pandas as pd
# import random
# import numpy as np
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
# from std_msgs.msg import Header
# from livox_ros_driver2.msg import CustomMsg


# class RANSAC:
#     def __init__(self, point_cloud, max_iterations=50, distance_threshold=0.2):
#         self.point_cloud = point_cloud
#         self.max_iterations = max_iterations
#         self.distance_threshold = distance_threshold

#     def run(self):
#         best_inliers = set()
#         for _ in range(self.max_iterations):
#             sample = self.point_cloud.sample(n=3)
#             x1, y1, z1 = sample.iloc[0]
#             x2, y2, z2 = sample.iloc[1]
#             x3, y3, z3 = sample.iloc[2]

#             # Plane coefficients
#             a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
#             b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
#             c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
#             d = -(a * x1 + b * y1 + c * z1)
#             norm = math.sqrt(a**2 + b**2 + c**2)
#             if norm == 0:
#                 continue  

#             inliers = set()
#             for idx, (x, y, z) in self.point_cloud.iterrows():
#                 distance = abs(a*x + b*y + c*z + d) / norm
#                 if distance <= self.distance_threshold:
#                     inliers.add(idx)


#             if len(inliers) > len(best_inliers):
#                 best_inliers = inliers

#         inlier_df = self.point_cloud.loc[list(best_inliers)].reset_index(drop=True)
#         return inlier_df


# # Global publisher
# plane_pub = None

# def livox_callback(msg: CustomMsg):
#     points = []
#     for pt in msg.points:
#         x, y, z = pt.x, pt.y, pt.z
#         if not any(map(math.isnan, (x, y, z))):
#             points.append((x, y, z))

#     if len(points) < 10:
#         rospy.logwarn("Not enough points to run RANSAC.")
#         return 

#     df = pd.DataFrame(points, columns=["X", "Y", "Z"])

#     # Calculate distances (just for logging/debug)
#     avg_distance = np.mean(np.linalg.norm(df[["X", "Y", "Z"]].values, axis=1))
#     rospy.loginfo(f"Frame received: {len(df)} points | Avg distance: {avg_distance:.2f} m")

#     # Run RANSAC
#     ransac = RANSAC(df)
#     plane_df = ransac.run()

#     # Publish inlier plane points
#     header = Header()
#     header.stamp = rospy.Time.now()
#     header.frame_id = msg.header.frame_id  # Same frame as original data
#     cloud = pc2.create_cloud_xyz32(header, plane_df[["X", "Y", "Z"]].values.tolist())
#     plane_pub.publish(cloud)
#     rospy.loginfo(f"Published {len(plane_df)} RANSAC inliers as plane")


# if __name__ == '__main__':
#     rospy.init_node('livox_ransac_plane_node')

#     plane_pub = rospy.Publisher('/ransac_plane', PointCloud2, queue_size=1)

#     rospy.Subscriber('/livox/lidar', CustomMsg, livox_callback)

#     rospy.loginfo("Listening to /livox/lidar and running RANSAC plane segmentation...")
#     rospy.spin()
