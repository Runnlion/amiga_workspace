
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def save_images_from_bag(bag_file, image_topic, output_dir):
    # Initialize the CvBridge to convert ROS Image messages to OpenCV format
    bridge = CvBridge()

    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Open the ROS bag
    with rosbag.Bag(bag_file, 'r') as bag:
        image_count = 0
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            if isinstance(msg, Image):
                # Convert the ROS Image message to an OpenCV image
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                except Exception as e:
                    rospy.logerr(f"Failed to convert image: {e}")
                    continue

                # Save the image as a file
                image_filename = os.path.join(output_dir, f"frame_{image_count:06d}.jpg")
                cv2.imwrite(image_filename, cv_image)
                rospy.loginfo(f"Saved image {image_filename}")

                image_count += 1

    rospy.loginfo(f"Finished extracting {image_count} images.")

if __name__ == '__main__':
    # Define bag file, image topic, and output directory
    bag_file = "/home/wolftech/lxiang3.lab/Desktop/sdu6/codes/amiga_workspace/2024-09-20-13-04-48.bag"       # Specify the path to your ROS bag file
    image_topic = "/camera_left/color/image_raw"          # Specify the image topic
    output_dir = "./extracted_images"          # Specify the directory to save images

    rospy.init_node('rosbag_image_extractor', anonymous=True)

    try:
        save_images_from_bag(bag_file, image_topic, output_dir)
    except rospy.ROSInterruptException:
        pass
