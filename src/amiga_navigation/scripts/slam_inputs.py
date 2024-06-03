#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, TransformStamped, Point, Quaternion, PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix
from tf.transformations import quaternion_from_matrix
import numpy as np

class PoseAndTransformListener:
    def __init__(self):
        self.base_link_pose = None

        # Initialize ROS node
        rospy.init_node('pose_and_tf_listener_node', anonymous=True)

        # Setup tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Read the transformation 
        self.velodyne_base = None
        self.velodyne_gazebo = None
        self.velodyne_base_rotation_mtx = None
        self.get_transform()


        
        # Subscribe to /gazebo/link_states
        self.seq = 0
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)
        self.velodync_gt_pub = rospy.Publisher('/ground_truth/velodyne',PoseStamped,queue_size=1)


        



    def link_states_callback(self, msg:LinkStates)->None:
        try:
            # Get the index of the base_link
            index = msg.name.index('amiga::base_link')
            # Get the pose of the base_link
            self.base_link_pose:Pose = msg.pose[index]
            # rospy.loginfo("Base Link Pose: %s", self.base_link_pose)
            # Help me to multiply given "self.velodyne_base" and "self.base_link_pose"
            rotation_mtx = quaternion_matrix([self.base_link_pose.orientation.x,
                                             self.base_link_pose.orientation.y,
                                             self.base_link_pose.orientation.z,
                                             self.base_link_pose.orientation.w])
            rotation_mtx[0:3,3] = np.array([self.base_link_pose.position.x,
                                              self.base_link_pose.position.y,
                                              self.base_link_pose.position.z])
            self.velodyne_gazebo = np.matmul(self.velodyne_base_rotation_mtx, rotation_mtx)
            velodyne_pose = Pose()
            velodyne_pose.position = Point(self.velodyne_gazebo[0,3], self.velodyne_gazebo[1,3], self.velodyne_gazebo[2,3])
            quaternion = quaternion_from_matrix(self.velodyne_gazebo)
            velodyne_pose.orientation = Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
            velodyne_pose_stamped = PoseStamped()
            velodyne_pose_stamped.pose = velodyne_pose
            velodyne_pose_stamped.header.frame_id = "world"
            velodyne_pose_stamped.header.stamp = rospy.Time.now()
            velodyne_pose_stamped.header.seq = self.seq
            self.velodync_gt_pub.publish(velodyne_pose_stamped)
            self.seq = self.seq + 1
            if(self.seq%1000 == 0):
                rospy.loginfo(f"Published {self.seq} Transformation Info.")
        except ValueError:
            rospy.logwarn("base_link not found in link_states")

    def get_transform(self)->bool:
        while not rospy.is_shutdown():
            try:
                # Lookup the transform from 'base_link' to 'velodyne'
                transform:TransformStamped = self.tf_buffer.lookup_transform('base_link', 'velodyne', rospy.Time(0), rospy.Duration(1.0))
                self.velodyne_base:TransformStamped = transform
                # print(type(self.velodyne_base))
                # rospy.loginfo("Transform from 'base_link' to 'velodyne': %s", transform)

                self.velodyne_base_rotation_mtx = quaternion_matrix([self.velodyne_base.transform.rotation.x,
                                            self.velodyne_base.transform.rotation.y,
                                            self.velodyne_base.transform.rotation.z,
                                            self.velodyne_base.transform.rotation.w])
                self.velodyne_base_rotation_mtx[0:3,3] = np.array([self.velodyne_base.transform.translation.x,
                                                    self.velodyne_base.transform.translation.y,
                                                    self.velodyne_base.transform.translation.z])
                rospy.loginfo("Transformation Received.")
                print(self.velodyne_base_rotation_mtx)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Transform not found. Retrying...")
                continue
        return True

if __name__ == '__main__':
    try:
        listener = PoseAndTransformListener()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
