import rospy
from geometry_msgs.msg import Twist, TwistStamped, Pose
from nav_msgs.msg import Odometry
import tf
import math

class UGVController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/twist_marker_server/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_stamped_pub = rospy.Publisher("/cmd_vel_stamped", TwistStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/pose_error", Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.target_pose = Pose()
        self.current_pose = Pose()
        
        self.Kp_p = 1.0
        self.Ki_p = 0.0
        self.Kd_p = 0.2
        
        self.Kp_theta = 1.0
        self.Ki_theta = 0
        self.Kd_theta = 0.2
        
        self.prev_error_p = 0.0
        self.integral_p = 0.0
        
        self.prev_error_theta = 0.0
        self.integral_theta = 0.0
        self.hertz:int = 100
        self.rate = rospy.Rate(self.hertz)
        self.time_interval:float = 1/self.hertz
        # Limits for linear and angular velocities
        self.max_linear_vel = 1.0   # maximum linear velocity (m/s)
        self.min_linear_vel = -1.0  # minimum linear velocity (m/s)
        self.max_angular_vel = 1.0  # maximum angular velocity (rad/s)
        self.min_angular_vel = -1.0 # minimum angular velocity (rad/s)
        self.linear_override = False
        self.finished = False

    def odom_callback(self, data):
        self.current_pose = data.pose.pose
    
    def set_target_pose(self, pose):
        self.target_pose = pose
    
    def compute_control(self):
        # Compute position error
        error_p = math.sqrt((self.target_pose.position.x - self.current_pose.position.x) ** 2 +
                            (self.target_pose.position.y - self.current_pose.position.y) ** 2)

        # Compute orientation error
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        _, _, yaw_current = tf.transformations.euler_from_quaternion(quaternion)

        if(abs(error_p)<=0.03):
            self.finished = True
            return
        else:
            # Compute desired heading
            desired_yaw = math.atan2(self.target_pose.position.y - self.current_pose.position.y,
                                    self.target_pose.position.x - self.current_pose.position.x)
            if(abs(desired_yaw - yaw_current)> math.pi):
                if(desired_yaw < 0):
                    desired_yaw = desired_yaw + math.pi * 2 
                if(yaw_current<0):
                    desired_yaw = desired_yaw - math.pi * 2 
            error_theta = desired_yaw - yaw_current

        print("position error:", error_p, "error_theta", error_theta, "target angle:", desired_yaw, "current angle:", yaw_current)
        error_pose = Twist()
        error_pose.linear.x = error_p
        error_pose.angular.z = error_theta
        self.error_pub.publish(error_pose)


        # PID for linear velocity
        self.integral_p += error_p * self.time_interval  # assuming loop runs at 10Hz
        derivative_p = (error_p - self.prev_error_p) / self.time_interval
        linear_vel = (self.Kp_p * error_p) + (self.Ki_p * self.integral_p) + (self.Kd_p * derivative_p)
        
        # PID for angular velocity
        self.integral_theta += error_theta * self.time_interval
        derivative_theta = (error_theta - self.prev_error_theta) / self.time_interval
        angular_vel = (self.Kp_theta * error_theta) + (self.Ki_theta * self.integral_theta) + (self.Kd_theta * derivative_theta)
        
        # Update previous errors
        self.prev_error_p = error_p
        self.prev_error_theta = error_theta

        # Apply command limits
        linear_vel = max(min(linear_vel, self.max_linear_vel), self.min_linear_vel)
        angular_vel = max(min(angular_vel, self.max_angular_vel), self.min_angular_vel)
        if(abs(error_theta) > math.pi /5 or (abs(error_theta) > math.pi * 4 / 5) ):
            linear_vel = 0.0
        # if(self.linear_override):
        #     linear_vel = 0.0
        #     angular_vel = 0.0

        # Create Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        cmd_vel_stamped = TwistStamped()
        cmd_vel_stamped.header.stamp = rospy.Time.now()
        cmd_vel_stamped.twist = cmd_vel
        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)
        self.cmd_vel_stamped_pub.publish(cmd_vel_stamped)
        # print(cmd_vel)

    ###
    ### Add a angle controller, if the target is -3.02, and current is 3.08. The UGV should move using shortest way
    def compute_control_angle(self):
        # Compute position error
        error_p = math.sqrt((self.target_pose.position.x - self.current_pose.position.x) ** 2 +
                            (self.target_pose.position.y - self.current_pose.position.y) ** 2)
        
        # Compute orientation error
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        _, _, yaw_current = tf.transformations.euler_from_quaternion(quaternion)
        
        quaternion_target = (
            self.target_pose.orientation.x,
            self.target_pose.orientation.y,
            self.target_pose.orientation.z,
            self.target_pose.orientation.w)
        _, _, yaw_target = tf.transformations.euler_from_quaternion(quaternion_target)

        if(abs(yaw_target - yaw_current)> math.pi):
            if(yaw_target < 0):
                yaw_target = yaw_target + math.pi * 2 
            if(yaw_current<0):
                yaw_target = yaw_target - math.pi * 2 
        error_theta = yaw_target - yaw_current

        # PID for linear velocity
        self.integral_p += error_p * 0.1  # assuming loop runs at 10Hz
        derivative_p = (error_p - self.prev_error_p) / 0.1
        linear_vel = (self.Kp_p * error_p) + (self.Ki_p * self.integral_p) + (self.Kd_p * derivative_p)
        
        # PID for angular velocity
        self.integral_theta += error_theta * 0.1
        derivative_theta = (error_theta - self.prev_error_theta) / 0.1
        angular_vel = (self.Kp_theta * error_theta) + (self.Ki_theta * self.integral_theta) + (self.Kd_theta * derivative_theta)
        
        # Update previous errors
        self.prev_error_p = error_p
        self.prev_error_theta = error_theta
        
        # Apply command limits
        linear_vel = max(min(linear_vel, self.max_linear_vel), self.min_linear_vel)
        angular_vel = max(min(angular_vel, self.max_angular_vel), self.min_angular_vel)
        
        # Create Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = angular_vel
        cmd_vel_stamped = TwistStamped()
        cmd_vel_stamped.header.stamp = rospy.Time.now()
        cmd_vel_stamped.twist = cmd_vel
        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)
        self.cmd_vel_stamped_pub.publish(cmd_vel_stamped)
        # print(cmd_vel)
        
        # Publish the command
        self.cmd_vel_pub.publish(cmd_vel)
        print(f"angle command: {cmd_vel.angular.z}\terror_theta: {error_theta}\tTraget/Current angle: {yaw_target}/{yaw_current}")
        error_pose = Twist()
        error_pose.linear.x = error_p
        error_pose.angular.z = error_theta
        self.error_pub.publish(error_pose)

    def run(self):
        while not rospy.is_shutdown():
            if(not self.finished):
                self.compute_control()
                # self.compute_control_angle()
                self.rate.sleep()
            else:
                self.compute_control_angle()
                # cmd_vel = Twist()
                # cmd_vel.linear.x = 0
                # cmd_vel.angular.z = 0
                # self.cmd_vel_pub.publish(cmd_vel)
                # rospy.loginfo("Finished.")
                self.rate.sleep()



if __name__ == '__main__':
    rospy.init_node('ugv_controller')
    controller = UGVController()
    
    # Set the target pose
    target_pose = Pose()
    target_pose.position.x = 2
    target_pose.position.y = 2
    quaternion = tf.transformations.quaternion_from_euler(0, 0, math.pi)
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]
    
    controller.set_target_pose(target_pose)
    controller.run()
    # rosbag record /cmd_vel_stamped /joint_states /odom /imu
