#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

def publish_path():
    rospy.init_node('path_publisher', anonymous=True)
    pub = rospy.Publisher('/path_topic', Path, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # 你的点集（假设为 (x, y, z) 坐标列表）
    points = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 1.0, 0.0),
        (3.0, 2.0, 0.0),
    ]

    path_msg = Path()
    path_msg.header.frame_id = "map"  # 根据你的坐标系修改

    for pt in points:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = pt[2]
        # 方向单位四元数（无旋转）
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()
        pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass