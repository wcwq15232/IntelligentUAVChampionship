#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu

R_ned_to_enu = np.array([[1, 0, 0],
                          [0, -1, 0],
                          [0, 0, -1]])

def imu_callback(msg):
    imu_enu = Imu()
    imu_enu.header = msg.header
    imu_enu.header.frame_id = "imu_enu"

    # 线性加速度
    acc = np.array([msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z])
    acc_enu = R_ned_to_enu @ acc
    imu_enu.linear_acceleration.x = acc_enu[0]
    imu_enu.linear_acceleration.y = acc_enu[1]
    imu_enu.linear_acceleration.z = acc_enu[2]
    imu_enu.linear_acceleration_covariance = msg.linear_acceleration_covariance

    # 角速度
    gyr = np.array([msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z])
    gyr_enu = R_ned_to_enu @ gyr
    imu_enu.angular_velocity.x = gyr_enu[0]
    imu_enu.angular_velocity.y = gyr_enu[1]
    imu_enu.angular_velocity.z = gyr_enu[2]
    imu_enu.angular_velocity_covariance = msg.angular_velocity_covariance
    
    imu_enu.orientation.x = msg.orientation.x
    imu_enu.orientation.y = msg.orientation.y
    imu_enu.orientation.z = msg.orientation.z
    imu_enu.orientation.w = msg.orientation.w
    imu_enu.orientation_covariance = [0] * 9  # 全部置 -1

    pub.publish(imu_enu)

if __name__ == '__main__':
    rospy.init_node('imu_converter')
    rospy.Subscriber('/airsim_node/drone_1/imu/imu', Imu, imu_callback)
    pub = rospy.Publisher('/imu/data_enu', Imu, queue_size=10)
    rospy.spin()