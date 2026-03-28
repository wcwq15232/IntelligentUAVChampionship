#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>

ros::Publisher pub;
Eigen::Affine3f transform;

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud_out;
    // 修正：使用 transform.matrix() 转换为 Matrix4f
    pcl_ros::transformPointCloud(transform.matrix(), *msg, cloud_out);
    cloud_out.header.frame_id = "lidar_enu";
    pub.publish(cloud_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_converter");
    ros::NodeHandle nh;

    // 构建变换矩阵：NED -> ENU
    transform = Eigen::Affine3f::Identity();
    transform.linear() << 1, 0, 0, 0, -1, 0, 0, 0, -1;

    ros::Subscriber sub = nh.subscribe("/airsim_node/drone_1/lidar", 10, lidar_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/points_enu", 10);
    ros::spin();
}