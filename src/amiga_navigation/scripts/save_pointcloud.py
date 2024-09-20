#!/usr/bin/env python

import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl_ros


import rospkg
from pathlib import Path
AMIGA_NAVIGATION_PKG_PATH = rospkg.RosPack().get_path('amiga_navigation')
VIDEO_PATH = AMIGA_NAVIGATION_PKG_PATH.replace('src/amiga_navigation','') + 'bag/test_videos/'
VIDEO_BAG_PATH = AMIGA_NAVIGATION_PKG_PATH.replace('src/amiga_navigation','') + 'bag/test_videos/pcds/'

def pointcloud_callback(msg):
    # Convert ROS PointCloud2 message to a PCL point cloud
    pc = pcl.PointCloud()
    points_list = []

    for point in pc2.read_points(msg, skip_nans=True):
        points_list.append([point[0], point[1], point[2]])

    # Create a PCL point cloud from the points list
    pc.from_list(points_list)

    # Save the point cloud to a .pcd file
    pcl.save(pc, VIDEO_BAG_PATH + 'output_pointcloud.pcd')
    rospy.loginfo("Point cloud saved to 'output_pointcloud.pcd'.")

if __name__ == '__main__':
    rospy.init_node('save_pointcloud_node', anonymous=True)

    # Replace with the correct point cloud topic (use 'rostopic list' to find the topic)
    pointcloud_topic = '/orb_slam3/all_points'

    # Subscribe to the PointCloud2 topic
    rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)

    rospy.spin()
