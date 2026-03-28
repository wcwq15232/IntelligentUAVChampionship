#!/usr/bin/env python3
import rospy
from path_sender.msg import WayPoints
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math

class WaypointsToPath:
    def __init__(self):
        rospy.init_node('waypoints_to_path', anonymous=True)
        
        # 发布MarkerArray，用于RViz显示多条路径
        self.marker_pub = rospy.Publisher('/path_markers', MarkerArray, queue_size=10)
        
        # 存储所有路径的点列表
        self.paths = []  # 每个元素是一个Point列表
        
        # 订阅/waypoints话题
        rospy.Subscriber('/waypoints', WayPoints, self.waypoints_callback)
        
        # 用于比较第一个点是否变化的上一个点
        self.last_first_point = None
        
        rospy.loginfo("Waypoints to Path converter started.")
    
    def points_equal(self, p1, p2):
        """判断两个Point是否相等（考虑浮点误差）"""
        eps = 1e-6
        return (abs(p1.x - p2.x) < eps and 
                abs(p1.y - p2.y) < eps and 
                abs(p1.z - p2.z) < eps)
    
    def waypoints_callback(self, msg):
        # 获取当前航点序列
        current_points = msg.points
        if not current_points:
            rospy.logwarn("Received empty waypoints list, ignoring.")
            return
        
        # 获取当前第一个点
        first_point = current_points[0]
        
        # 如果是第一个消息，直接添加路径
        if self.last_first_point is None:
            rospy.loginfo("First waypoints segment received, adding path with %d points.", len(current_points))
            self.paths.append(current_points)
            self.last_first_point = first_point
            self.publish_markers()
            return
        
        # 比较第一个点是否变化
        if not self.points_equal(first_point, self.last_first_point):
            rospy.loginfo("First point changed: new waypoints segment detected. Adding new path with %d points.", len(current_points))
            self.paths.append(current_points)
            self.last_first_point = first_point
            self.publish_markers()
        else:
            # 第一个点相同，忽略重复消息（每秒重复）
            rospy.logdebug("First point unchanged, ignoring duplicate message.")
    
    def publish_markers(self):
        """将存储的所有路径转换为MarkerArray并发布"""
        marker_array = MarkerArray()
        
        # 定义颜色表（每条路径不同颜色，可选）
        colors = [
            (1.0, 0.0, 0.0),  # 红
            (0.0, 1.0, 0.0),  # 绿
            (0.0, 0.0, 1.0),  # 蓝
            (1.0, 1.0, 0.0),  # 黄
            (1.0, 0.0, 1.0),  # 品红
            (0.0, 1.0, 1.0),  # 青
        ]
        
        for idx, points in enumerate(self.paths):
            marker = Marker()
            marker.header.frame_id = "odom"      # 假设坐标系为map，可根据实际情况修改
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # 设置线条宽度（米）
            marker.scale.x = 0.05
            
            # 设置颜色（循环使用颜色表）
            color = colors[idx % len(colors)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            # 添加所有点
            for p in points:
                point = Point()
                point.x = p.x
                point.y = p.y
                point.z = p.z
                marker.points.append(point)
            
            # 永久显示
            marker.lifetime = rospy.Duration(0)
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
        rospy.logdebug("Published %d path markers.", len(self.paths))

if __name__ == '__main__':
    try:
        node = WaypointsToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass