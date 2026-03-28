#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseGTProcessor:
    def __init__(self):
        rospy.init_node('pose_gt_processor')
        # 订阅原始pose_gt话题
        self.sub = rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped, self.callback)
        # 发布处理后的轨迹话题
        self.pub = rospy.Publisher('/airsim_node/drone_1/debug/pose_gt_path', Path, queue_size=1)
        # 存储轨迹点
        self.path = Path()
        self.path.header.frame_id = 'odom'
        # 轨迹点数量限制，避免内存占用过大
        self.max_points = 1000
        rospy.loginfo("Pose GT Processor node started")
    
    def callback(self, msg):
        # 创建新的PoseStamped消息
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose
        pose.pose.position.z = -pose.pose.position.z
        pose.pose.position.y = -pose.pose.position.y

        
        # 添加到轨迹中
        self.path.poses.append(pose)
        # 更新轨迹的时间戳
        self.path.header.stamp = rospy.Time.now()
        
        # 限制轨迹点数量
        if len(self.path.poses) > self.max_points:
            self.path.poses.pop(0)
        
        # 发布轨迹
        self.pub.publish(self.path)

if __name__ == '__main__':
    try:
        processor = PoseGTProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass