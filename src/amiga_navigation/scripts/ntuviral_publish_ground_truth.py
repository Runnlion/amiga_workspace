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

    def callback(self, msg: PoseStamped):
        if(self.init_position_achieved == False):
            self.init_x = msg.pose.position.x
            self.init_y = msg.pose.position.y
            self.init_z = msg.pose.position.z
            self.init_position_achieved = True
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "world"
        pose.pose.position.x = msg.pose.position.x
        pose.pose.position.y = msg.pose.position.y
        pose.pose.position.z = msg.pose.position.z
        self.path_msg.poses.append(pose)
        

def main():
    rospy.init_node('position_ground_truth_generator')
    generator = PositionGroundTruthGenerator()
    rospy.Subscriber('/leica/pose/relative', PoseStamped, generator.callback)
    while not rospy.is_shutdown():
        generator.path_pub.publish(generator.path_msg)
        rospy.loginfo(f"Number Data: {len(generator.path_msg.poses)}")
        rospy.Rate(10).sleep()
    rospy.spin()
    

if __name__ == '__main__':
    main()