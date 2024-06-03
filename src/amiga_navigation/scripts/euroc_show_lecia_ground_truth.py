import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point

class PositionGroundTruthGenerator:
    def __init__(self):
        self.path_pub = rospy.Publisher('/position_ground_truth', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.init_x = 0
        self.init_y = 0
        self.init_z = 0
        self.init_position_achieved = False

    def callback(self, msg: PointStamped):
        if(self.init_position_achieved == False):
            self.init_x = msg.point.x
            self.init_y = msg.point.y
            self.init_z = msg.point.z
            self.init_position_achieved = True
        point:Point = msg.point
        pose = PoseStamped()
        pose.header = msg.header

        # deal with offset data
        # pose.pose.position.x = point.x - self.init_x
        # pose.pose.position.y = point.y - self.init_y
        # pose.pose.position.z = point.z - self.init_z
        
        # rotate along z axis by -90 degrees
        # pose.pose.position.x = pose.pose.position.y
        # pose.pose.position.y = -pose.pose.position.x
        # pose.pose.position.z = pose.pose.position.z

        pose.pose.position.x = point.y - self.init_y
        pose.pose.position.y = -(point.x - self.init_x)
        pose.pose.position.z = point.z - self.init_z

        self.path_msg.poses.append(pose)

def main():
    rospy.init_node('position_ground_truth_generator')
    generator = PositionGroundTruthGenerator()
    rospy.Subscriber('/leica/position', PointStamped, generator.callback)
    while not rospy.is_shutdown():
        generator.path_pub.publish(generator.path_msg)
        rospy.loginfo(f"Number Data: {len(generator.path_msg.poses)}")
        rospy.Rate(10).sleep()
    rospy.spin()
    

if __name__ == '__main__':
    main()