#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <yolov11_ros_msgs/BoundingBoxes.h>
#include <yolov11_ros_msgs/BoundingBox.h>
#include <path_sender/WayPoints.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <array>
#include <deque>
#include <cmath>

cv::Mat kernel_7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
cv::Mat kernel_5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::Mat kernel_3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));


struct light
{
    light(const cv::Point& a, const cv::Point& b, float d) : p1(a), p2(b), length(d) {}
    cv::Point p1; // 高
    cv::Point p2; // 低
    float length;
};

struct light_pair
{
    light left;
    light right;
    float area;
};

class VisionNode{
public:
    VisionNode(ros::NodeHandle *nh);
    ~VisionNode();
private:
    bool odom_init;
    int count = 0;

    ros::Publisher pub_right_blur;
    ros::Publisher pub_left_blur;
    ros::Publisher waypoint_pub;
    cv::Mat left_img;
    cv::Mat right_img;

    nav_msgs::Odometry odom_msg;
    Eigen::Quaterniond pose_q;
    Eigen::Vector3d pose_t;
    Eigen::Matrix3d pose_R;
    ros::Timer timer;

    std::deque<Eigen::Vector3d> waypoints;
    std::deque<Eigen::Vector3d> finished_points;

    void img_pre_process(cv::Mat &img, cv::Mat &dst);
    std::vector<light_pair> get_doors(cv::Mat &imgDil, cv::Mat &img);
    void match_doors(std::vector<light_pair> left_doors, std::vector<light_pair> right_doors, std::vector<std::array<cv::Point, 8>> &matched_points);
    Eigen::Vector3d compute_center_point(std::array<cv::Point, 8> points);
    void img_callback(const sensor_msgs::ImageConstPtr &left_img_msg, const sensor_msgs::ImageConstPtr &right_img_msg);
    void yolo_cb(const yolov11_ros_msgs::BoundingBoxesConstPtr &msg);
    void odom_cb(const nav_msgs::OdometryConstPtr pMsg);
    void send_waypoints();
    void time_cb(const ros::TimerEvent &event);
};