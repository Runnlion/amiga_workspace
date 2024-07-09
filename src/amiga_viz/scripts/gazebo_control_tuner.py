import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped, Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import time

# Lists to store the data

timestamps_gps = []
gps_speed_x = []

timestamps_cmd = []
twist_linear_velocity_x = []
twist_angular_velocity = []

timestamps_imu = []
imu_angular_speed = []
# Create figure for plotting
fig = plt.figure()
ax_velocity = fig.add_subplot(2, 1, 1)
ax_angular = fig.add_subplot(2, 1, 2)

# Callback for the /navsat/vel topic
def navsat_callback(data: Vector3Stamped):
    global gps_speed_x, timestamps_gps
    gps_speed_x.append(data.vector.x)
    timestamps_gps.append(data.header.stamp.to_sec())
    rospy.loginfo("GPS velocity Received.")

# Callback for the /twist_marker_server/cmd_vel topic
def twist_callback(data: Twist):
    global twist_linear_velocity_x, twist_angular_velocity, timestamps_cmd
    sim_time = rospy.get_time()
    twist_linear_velocity_x.append(data.linear.x)
    twist_angular_velocity.append(data.angular.z)
    timestamps_cmd.append(sim_time)
    rospy.loginfo("Cmd Velocity Received.")

def imu_callback(data:Imu):
    timestamps_imu.append(data.header.stamp.to_sec())
    imu_angular_speed.append(data.angular_velocity.z)
    rospy.loginfo("IMU Message Received.")


# This function is called periodically from FuncAnimation
def animate(i, timestamps_cmd, twist_linear_velocity_x, timestamps_gps, gps_speed_x, timestamp_imu, imu_angular_speed, twist_angular_velocity):


    # Limit x and y lists to 20 items
    # xs = xs[-100:]
    # ys = ys[-100:]

    # Draw x and y lists
    ax_velocity.clear()
    ax_angular.clear()

    ax_velocity.scatter(timestamps_cmd, twist_linear_velocity_x ,label='Twist Linear Velocity (x)', color='red')
    ax_velocity.plot(timestamps_gps, gps_speed_x, label='GPS Speed (x)', color='blue')
    ax_velocity.grid()
    ax_velocity.legend(['Twist Linear Velocity (x)', 'GPS Speed (x)'])
    ax_velocity.set_title('Linear Velocity')

    ax_angular.scatter(timestamps_cmd, twist_angular_velocity ,label='Angular Velocity (z)', color='red')
    ax_angular.plot(timestamp_imu,imu_angular_speed,label='IMU Speed (x)', color='blue')
    ax_angular.grid()
    ax_angular.legend(['Twist Angular Velocity (z)', 'IMU Angular Speed (z)'])
    ax_angular.set_title('Angular Velocity')
    
    # Format plot
    # plt.xticks(rotation=45, ha='right')
    # plt.subplots_adjust(bottom=0.30)
    # plt.title('')
    # plt.ylabel('Linear Velocity')


def main():
    rospy.init_node('velocity_plotter', anonymous=True)

    rospy.Subscriber("/navsat/vel", Vector3Stamped, navsat_callback)
    rospy.Subscriber("/cmd_vel", Twist, twist_callback)
    rospy.Subscriber("/twist_marker_server/cmd_vel", Twist, twist_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)
    # rospy.Subscriber("/imu/data", Imu, imu_callback)

    rate = rospy.Rate(1)  # 10 Hz
    # print(twist_linear_velocity_x)
    # print(gps_speed_x)
    ani = animation.FuncAnimation(fig, animate, fargs=(timestamps_cmd, twist_linear_velocity_x, timestamps_gps, gps_speed_x, timestamps_imu, imu_angular_speed, twist_angular_velocity), interval=1000)
    plt.show()
    # rospy.spin()



if __name__ == '__main__':
    main()
