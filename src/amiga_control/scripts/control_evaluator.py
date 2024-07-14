import rosbag
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import TwistStamped, Pose, Twist
from nav_msgs.msg import Odometry
import tf.transformations
import math
# Function to extract data from rosbag
def extract_data_from_rosbag(bag_file):
    bag = rosbag.Bag(bag_file)
    
    # Lists to store data
    odom_positions = []
    odom_orientations = []
    cmd_velocities = []
    odom_velocities = []
    odom_timestamps = []
    cmd_timestamps = []
    cmd_position_error = []
    cmd_angular_error = []
    
    for topic, msg, t in bag.read_messages(topics=['/odom', '/cmd_vel_stamped', '/pose_error']):
        if topic == '/odom':
            Odom_Msg:Odometry = msg
            odom_positions.append((Odom_Msg.pose.pose.position.x, Odom_Msg.pose.pose.position.y))
            orientation = (Odom_Msg.pose.pose.orientation.x,
                           Odom_Msg.pose.pose.orientation.y,
                           Odom_Msg.pose.pose.orientation.z,
                           Odom_Msg.pose.pose.orientation.w)
            odom_orientations.append(orientation)
            odom_velocities.append((Odom_Msg.twist.twist.linear.x, Odom_Msg.twist.twist.angular.z))
            odom_timestamps.append(Odom_Msg.header.stamp.to_sec())
        elif topic == '/cmd_vel_stamped':
            cmd_stamped_msg:TwistStamped = msg
            cmd_velocities.append((cmd_stamped_msg.twist.linear.x, cmd_stamped_msg.twist.angular.z))
            cmd_timestamps.append(cmd_stamped_msg.header.stamp.to_sec())
        elif topic == '/pose_error':
            error_twist:Twist =  msg
            cmd_position_error.append(error_twist.linear.x)
            cmd_angular_error.append(error_twist.angular.z)

    
    bag.close()
    return odom_positions, odom_orientations, cmd_velocities, odom_velocities,  cmd_timestamps, odom_timestamps, cmd_position_error, cmd_angular_error

# Function to plot trajectory and directions
def plot_trajectory(odom_positions, odom_orientations, point1, point2):
    x, y = zip(*odom_positions)
    
    plt.figure(figsize=(10, 5))
    plt.plot(x, y, label='Trajectory')
    plt.scatter([point1[0]], [point1[1]], color='red', label='Start Points')
    plt.scatter([point2[0]], [point2[1]], color='blue', label='Target Points')
    for pos, ori in zip(odom_positions[::5], odom_orientations[::5]):
        yaw = tf.transformations.euler_from_quaternion(ori)[2]
        plt.arrow(pos[0], pos[1], 0.1 * np.cos(yaw), 0.1 * np.sin(yaw), head_width=0.03)
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('UGV Trajectory and Direction')
    # plt.ylim([0,20])
    # plt.xlim([0,20])
    plt.legend()
    plt.grid()
    # plt.show()

# Function to plot velocity comparison
def plot_velocity_comparison(cmd_velocities, odom_velocities, odom_orientations, cmd_timestamps, odom_timestamps, cmd_angular_errorcmd_position_error, cmd_angular_error):

    odom_yaw = []
    for _odom in odom_orientations:
        odom_yaw.append(tf.transformations.euler_from_quaternion(_odom)[2] / math.pi * 180.0 )

    cmd_linear, cmd_angular = zip(*cmd_velocities)
    odom_linear, odom_angular = zip(*odom_velocities)
    
    # cmd_position_error.append(cmd_position_error[-1])
    # cmd_angular_error.append(cmd_angular_error[-1])

    plt.figure(figsize=(15, 7))
    plt.subplot(3, 1, 1)
    plt.plot(cmd_timestamps, cmd_linear, label='cmd_vel Linear Velocity', color="blue")
    plt.plot(odom_timestamps, odom_linear, label='odom Linear Velocity', linestyle='--')
    plt.plot(cmd_timestamps, cmd_position_error, label='positional Error', color="red")
    # plt.plot(cmd_timestamps[0,len(cmd_timestamps)-2], cmd_position_error, label='positional Error', color="red")

    plt.xlabel('Time')
    plt.ylabel('Linear Velocity (m/s)')
    plt.legend()
    plt.grid()
    
    plt.subplot(3, 1, 2)
    plt.plot(cmd_timestamps, cmd_angular, label='cmd_vel Angular Velocity',  color="blue")
    plt.plot(odom_timestamps, odom_angular, label='odom Angular Velocity', linestyle='--')
    plt.plot(cmd_timestamps, cmd_angular_error, label='Angular Error', color="red")
    # plt.plot(cmd_timestamps[0,len(cmd_timestamps)-2], cmd_angular_error, label='Angular Error', color="red")

    plt.xlabel('Time')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid()


    plt.subplot(3, 1, 3)
    plt.plot(odom_timestamps, odom_yaw, label='UGV Heading Ground Truth')
    plt.xlabel('Time')
    plt.ylabel('Heading (Degree)')
    plt.legend()
    plt.grid()

    plt.suptitle('Velocity Comparison')
    plt.show()

# Main function
if __name__ == "__main__":
    import tf
    
    # Path to the rosbag file
    # bag_file = '/mnt/Data/amiga_workspace/bag/PID/success_report_case_2.bag'
    bag_file = '/mnt/Data/amiga_workspace/bag/PID/0716/2024-07-14-04-49-09.bag'
    
    # Given points
    point1 = (0,0)
    point2 = (5,5)
    
    # Case 3 -8,-13
    # Extract data from rosbag
    odom_positions, odom_orientations, cmd_velocities, odom_velocities, cmd_timestamps, odom_timestamps, cmd_position_error, cmd_angular_error = extract_data_from_rosbag(bag_file)
    
    # Plot trajectory and direction
    plot_trajectory(odom_positions, odom_orientations, point1, point2)
    
    # Plot velocity comparison
    plot_velocity_comparison(cmd_velocities, odom_velocities, odom_orientations, cmd_timestamps, odom_timestamps, cmd_position_error, cmd_angular_error)
