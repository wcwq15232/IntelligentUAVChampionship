#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_multiply, quaternion_from_matrix

class GpsToOdometry:
    def __init__(self):
        rospy.init_node('gps_converter')
        
        # 旋转矩阵与四元数（NED->ENU）
        self.R_ned_to_enu = np.array([[1, 0, 0],
                                       [0, -1, 0],
                                       [0, 0, -1]])
        # 对应四元数 (绕X轴180°)
        self.q_rot = [0, 1, 0, 0]  # w,x,y,z

        self.origin_enu = None      # 起始点ENU坐标
        self.origin_set = False

        # 订阅起始位姿（假设只发布一次）
        rospy.Subscriber('/airsim_node/initial_pose', PoseStamped, self.init_callback)

        # 订阅GPS
        rospy.Subscriber('/airsim_node/drone_1/gps', PoseStamped, self.gps_callback)

        # 发布转换后的GPS里程计
        self.odom_pub = rospy.Publisher('/odometry/gps_enu', Odometry, queue_size=10)

        rospy.spin()

    def init_callback(self, msg):
        if self.origin_set:
            return
        # 将起始点从NED转换到ENU
        pos_ned = np.array([msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z])
        self.origin_enu = self.R_ned_to_enu @ pos_ned
        self.origin_set = True
        rospy.loginfo("GPS origin set: [%.2f, %.2f, %.2f] (ENU)",
                      self.origin_enu[0], self.origin_enu[1], self.origin_enu[2])

    def gps_callback(self, msg):
        if not self.origin_set:
            rospy.logwarn_throttle(5, "Waiting for initial pose...")
            return

        # 1. 位置转换
        pos_ned = np.array([msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z])
        pos_enu = self.R_ned_to_enu @ pos_ned
        pos_local = pos_enu - self.origin_enu

        # 2. 姿态转换
        q_ned = [msg.pose.orientation.w,
                 msg.pose.orientation.x,
                 msg.pose.orientation.y,
                 msg.pose.orientation.z]
        q_enu = quaternion_multiply(self.q_rot, q_ned)  # 注意顺序：q_rot * q_ned

        # 3. 构建Odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"  # 可忽略

        odom.pose.pose.position.x = pos_local[0]
        odom.pose.pose.position.y = pos_local[1]
        odom.pose.pose.position.z = pos_local[2]

        odom.pose.pose.orientation.w = q_enu[0]
        odom.pose.pose.orientation.x = q_enu[1]
        odom.pose.pose.orientation.y = q_enu[2]
        odom.pose.pose.orientation.z = q_enu[3]

        # 设置协方差 (位置方差0.01 m²，姿态方差0.04 rad²)
        # 6x6协方差矩阵，按行填充 [x,y,z,roll,pitch,yaw]
        cov = np.zeros(36)
        cov[0] = 0.01   # x
        cov[7] = 0.01   # y
        cov[14] = 0.01  # z
        cov[21] = 0.04  # roll
        cov[28] = 0.04  # pitch
        cov[35] = 0.04  # yaw
        odom.pose.covariance = cov.tolist()

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    GpsToOdometry()