#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rosbag
import os


def video_pub(video_path, topic_name='/cam0/image_raw', frame_rate=30):
    # Initialize ROS node
    rospy.init_node('video_to_bag_node', anonymous=True)

    image_pub = rospy.Publisher(topic_name,Image,queue_size=1)
    # Create a CvBridge to convert OpenCV images to ROS messages
    bridge = CvBridge()

    # Open the video file
    cap = cv2.VideoCapture(video_path)
    # Create a publisher for the CameraInfo message
    camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    
    # Define the CameraInfo message
    camera_info_msg = CameraInfo()
    
    # Set the image width and height
    camera_info_msg.width = 3840
    camera_info_msg.height = 2160

    # Set the distortion model and coefficients
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0]  # Including k1, k2, p1, p2, and k3=0

    # Intrinsic camera matrix (K)
    # [fx,  0, cx]
    # [ 0, fy, cy]
    # [ 0,  0,  1]
    camera_info_msg.K = [668.145, 0.0, 367.215,
                         0.0, 644.934, 248.375,
                         0.0, 0.0, 1.0]
    
    # Rectification matrix (R)
    camera_info_msg.R = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]  # No rectification, identity matrix
    
    # Projection matrix (P)
    # [fx,  0, cx, Tx]
    # [ 0, fy, cy, Ty]
    # [ 0,  0,  1,  0]
    camera_info_msg.P = [668.145, 0.0, 367.215, 0.0,
                         0.0, 644.934, 248.375, 0.0,
                         0.0, 0.0, 1.0, 0.0]
    
    # Frame ID (replace with your camera's frame)
    camera_info_msg.header.frame_id = "camera"

    # Get the video frame rate (if not provided)
    if frame_rate is None:
        frame_rate = cap.get(cv2.CAP_PROP_FPS)

    # Open the bag file for writing
    try:
        # Initialize the frame count and time
        frame_count = 0
        time_increment = rospy.Duration(1.0 / frame_rate)
        current_time = rospy.Time.now()
        # Set the current time as the timestamp
        
        # Publish the CameraInfo message
        while cap.isOpened():
            ret, frame = cap.read()
            resized_frame = cv2.resize(frame, (1920, 1080))
            if not ret:
                break

            # Convert the OpenCV frame to a ROS Image message
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Set the header timestamp
            img_msg.header.stamp = current_time
            img_msg.header.frame_id = 'camera'
            image_pub.publish(img_msg)
            camera_info_msg.header.stamp = current_time
            camera_info_pub.publish(camera_info_msg)

            # Increment the time and frame count
            current_time += time_increment
            frame_count += 1
            print(img_msg.height, img_msg.width)
            rospy.loginfo("Frame Publushed %d", frame_count)
            rospy.Rate(10).sleep()
            if(rospy.is_shutdown()):
                quit()
    finally:
        # Release the video and bag file resources
        cap.release()


import rospkg
from pathlib import Path
AMIGA_NAVIGATION_PKG_PATH = rospkg.RosPack().get_path('amiga_navigation')
VIDEO_PATH = AMIGA_NAVIGATION_PKG_PATH.replace('src/amiga_navigation','') + 'bag/test_videos/'
VIDEO_BAG_PATH = AMIGA_NAVIGATION_PKG_PATH.replace('src/amiga_navigation','') + 'bag/test_videos/bags/'
if __name__ == '__main__':
    # Replace with your video and bag file paths
    video_path = VIDEO_PATH + '1.mp4'

    try:
        video_pub(video_path)
    except rospy.ROSInterruptException:
        pass
