import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from geographiclib.geodesic import Geodesic
# from geodesy.utm import UTMPoint
from geodesy import utm

class PositionGroundTruthGenerator:
    def __init__(self):
        self.path_pub = rospy.Publisher('/position_ground_truth', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.init_x = 0
        self.init_y = 0
        self.init_z = 0
        self.init_position_achieved = False

    def navsatfix_to_point(self, navsatfix:NavSatFix)->Point:
        # Convert latitude and longitude to UTM
        utm_point = utm.fromLatLong(navsatfix.latitude, navsatfix.longitude)
        # utm.UTMPoint.toMsg(utm_point)
        # Create Point message
        point = Point()
        point.x = utm_point.easting
        point.y = utm_point.northing
        point.z = navsatfix.altitude  # use altitude directly as z
        return point


    def callback(self, msg: NavSatFix):
        point:Point = self.navsatfix_to_point(msg)
        if(self.init_position_achieved == False):
            self.init_x = point.x
            self.init_y = point.y
            self.init_z = point.z
            self.init_position_achieved = True
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "world"
        pose.pose.position.x = point.x - self.init_x
        pose.pose.position.y = point.y - self.init_y
        pose.pose.position.z = point.z - self.init_z
        self.path_msg.poses.append(pose)

def main():
    rospy.init_node('position_ground_truth_generator')
    generator = PositionGroundTruthGenerator()
    rospy.Subscriber('/kitti/oxts/gps/fix', NavSatFix, generator.callback)
    while not rospy.is_shutdown():
        generator.path_pub.publish(generator.path_msg)
        rospy.loginfo(f"Number Data: {len(generator.path_msg.poses)}")
        rospy.Rate(10).sleep()
    rospy.spin()
    

if __name__ == '__main__':
    main()