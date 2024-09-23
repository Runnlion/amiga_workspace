

#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def get_transform():
    rospy.init_node('tf_listener_node', anonymous=True)
    
    # Create a tf listener
    listener = tf.TransformListener()

    # Define the target frame you want to get the pose for (e.g., base_link relative to world)
    target_frame = 'camera_left_color_optical_frame'  # Change this to the desired frame
    reference_frame = 'body_link'   # This is usually the world or map frame in Gazebo

    # Ensure that the transform is available
    listener.waitForTransform(reference_frame, target_frame, rospy.Time(), rospy.Duration(4.0))
    
    try:
        # Get the latest available transform between the target frame and reference frame
        (trans, rot) = listener.lookupTransform(reference_frame, target_frame, rospy.Time(0))
        
        # Translation (position)
        rospy.loginfo(f"Translation (position): x={trans[0]}, y={trans[1]}, z={trans[2]}")

        # Rotation (orientation)
        rospy.loginfo(f"Rotation (orientation): x={rot[0]}, y={rot[1]}, z={rot[2]}, w={rot[3]}")
    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Error while fetching transform: {e}")
    
if __name__ == '__main__':
    try:
        get_transform()
    except rospy.ROSInterruptException:
        pass

