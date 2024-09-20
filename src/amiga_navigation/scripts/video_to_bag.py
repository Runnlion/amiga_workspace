#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rosbag
import os

def video_to_bag(video_path, bag_path, topic_name='/camera/image_raw', frame_rate=30):
    # Initialize ROS node
    rospy.init_node('video_to_bag_node', anonymous=True)

    # Create a CvBridge to convert OpenCV images to ROS messages
    bridge = CvBridge()

    # Open the video file
    cap = cv2.VideoCapture(video_path)

    # Get the video frame rate (if not provided)
    if frame_rate is None:
        frame_rate = cap.get(cv2.CAP_PROP_FPS)

    # Open the bag file for writing
    bag = rosbag.Bag(bag_path, 'w')

    try:
        # Initialize the frame count and time
        frame_count = 0
        time_increment = rospy.Duration(1.0 / frame_rate)
        current_time = rospy.Time.now()

        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                break

            # Convert the OpenCV frame to a ROS Image message
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Set the header timestamp
            img_msg.header.stamp = current_time

            # Write the message to the bag file
            bag.write(topic_name, img_msg, current_time)

            # Increment the time and frame count
            current_time += time_increment
            frame_count += 1

            rospy.loginfo("Processed frame %d", frame_count)

    finally:
        # Release the video and bag file resources
        cap.release()
        bag.close()

    rospy.loginfo("Finished writing the video to bag file: %s", bag_path)

import rospkg
from pathlib import Path
AMIGA_NAVIGATION_PKG_PATH = rospkg.RosPack().get_path('amiga_navigation')
VIDEO_PATH = AMIGA_NAVIGATION_PKG_PATH.replace('src/amiga_navigation','') + 'bag/test_videos/'
VIDEO_BAG_PATH = AMIGA_NAVIGATION_PKG_PATH.replace('src/amiga_navigation','') + 'bag/test_videos/bags/'
if __name__ == '__main__':
    # Replace with your video and bag file paths
    video_path = VIDEO_PATH + '1.mp4'
    bag_path = VIDEO_BAG_PATH + "1.bag"

    try:
        video_to_bag(video_path, bag_path)
    except rospy.ROSInterruptException:
        pass
