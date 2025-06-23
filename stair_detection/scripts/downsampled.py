#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2

pub = None

def callback(msg):
    # Convert PointCloud2 to numpy array
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    xyz = np.array([p for p in points], dtype=np.float32)

    if len(xyz) == 0:
        return

    # Create Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # Downsample
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.05)
    down_xyz = np.asarray(downsampled_pcd.points)

    # Convert back to PointCloud2
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id

    down_msg = pcl2.create_cloud_xyz32(header, down_xyz.tolist())
    pub.publish(down_msg)

if __name__ == '__main__':
    rospy.init_node('downsample_livox_node')
    pub = rospy.Publisher('/livox/xz_downsampled', PointCloud2, queue_size=1)
    rospy.Subscriber('/livox/xz_plane', PointCloud2, callback)
    rospy.loginfo("Downsampling /livox/xz_plane and publishing to /livox/xz_downsampled...")
    rospy.spin()
