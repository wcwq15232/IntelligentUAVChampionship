#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from path_sender.msg import WayPoints  # 自定义消息类型

class WaypointsVisualizer:
    def __init__(self):
        rospy.init_node('waypoints_visualizer', anonymous=True)
        
        # 发布路径话题，用于RViz显示
        self.path_pub = rospy.Publisher('/path_visualization', Path, queue_size=1, latch=True)
        
        # 订阅航点话题
        rospy.Subscriber('/waypoints', WayPoints, self.waypoints_callback)
        
        rospy.loginfo("Waypoints Visualizer 已启动，等待 /waypoints 消息...")

    def waypoints_callback(self, msg):
        """
        回调函数：将接收到的航点转换为Path消息并发布
        """
        # 确定坐标系（通常为map或world，根据实际环境调整）
        frame_id = "odom"  # 根据你的TF树修改，例如"world"
        
        # 创建Path消息
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = frame_id
        
        # 遍历所有点，构造PoseStamped
        for point in msg.points:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = frame_id
            pose.pose.position = point  # point已经是geometry_msgs/Point
            pose.pose.orientation.w = 1.0  # 单位四元数（无旋转）
            
            path_msg.poses.append(pose)
        
        # 发布路径
        self.path_pub.publish(path_msg)
        rospy.loginfo("已发布包含 %d 个点的路径", len(msg.points))

if __name__ == '__main__':
    try:
        wv = WaypointsVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass