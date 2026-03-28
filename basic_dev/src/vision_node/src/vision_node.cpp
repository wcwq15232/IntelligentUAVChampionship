#include "vision_node.hpp"
#include <cmath>

using namespace cv;
using namespace std;

double Base = 0.3; // 相机基线长度
double F = 831.3843994140625;
double Cx = 480, Cy = 360; 

//   l1               l2  r1               r2
//
//
//
//   l4               l3  r4               r3

// l1 l2 l3 l4 r1 r2 r3 r4

ros::Publisher pub_right_blur;
ros::Publisher pub_left_blur;


Mat left_img;
Mat right_img;

nav_msgs::Odometry odom_msg;
Eigen::Quaterniond pose_q;
Eigen::Vector3d pose_t;
Eigen::Matrix3d pose_R;

float distance(const Point &a, const Point &b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;

    // cout << "dis" << sqrt(dx * dx + dy * dy) << endl;
    return std::sqrt(dx * dx + dy * dy);
}

inline float distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return (a - b).norm();
}


void VisionNode::img_pre_process(Mat &img, Mat &dst)
{
    vector<Mat> channels;
    Mat binary;

    split(img, channels);

    threshold(channels[2] - channels[1], binary, 28, 255, THRESH_BINARY);
    dilate(binary, dst, kernel_3);
}


vector<light_pair> VisionNode::get_doors(Mat &imgDil, Mat &img)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<light> lights;

    findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // drawContours(img, contours, -1, Scalar(255, 0, 255), 2);

    vector<float> areas(contours.size());
    vector<vector<Point>> conPoly(contours.size());

    for (int i = 0; i < contours.size(); ++i) {
        areas[i] = (float)contourArea(contours[i]);
    }
    
    for (int i = 0; i < contours.size(); ++i)
    {
        if (areas[i] < 100)
        {
            continue;
        }

        float peri = arcLength(contours[i], true);
        approxPolyDP(contours[i], conPoly[i], 0.03 * peri, true);
        // drawContours(img, conPoly, i, Scalar(0, 0, 255), 2, LINE_4);

        if (conPoly[i].size() == 2)
        {

            double angle = abs(atan2(conPoly[i][0].x - conPoly[i][1].x, conPoly[i][0].y - conPoly[i][1].y) / M_PI);
            // cout << angle << endl;
            if (angle < 0.75)
                continue;

            // putText(img, to_string(angle), (conPoly[i][0] + conPoly[i][1]) / 2, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 1);
            light tmp_l = (conPoly[i][0].y > conPoly[i][1].y)
            ? light(conPoly[i][0], conPoly[i][1], distance(conPoly[i][0], conPoly[i][1]))
            : light(conPoly[i][1], conPoly[i][0], distance(conPoly[i][0], conPoly[i][1]));

            lights.push_back(tmp_l);

            // cout << "长度： "<< lights.back().length << endl;
            putText(img, to_string(lights.back().length), (conPoly[i][0] + conPoly[i][1]) / 2, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 1);

            line(img, conPoly[i][0], conPoly[i][1], Scalar(0, 255, 0), 2);
            circle(img, conPoly[i][0], 5, Scalar(255, 0, 0), 3);
            circle(img, conPoly[i][1], 5, Scalar(255, 0, 0), 3);
        }
        // boundRect[i] = boundingRect(conPoly[i]);
        // rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2);
    }

    vector<light_pair> doors;

    if (lights.size() >= 2)
    {
        sort(lights.begin(), lights.end(), [](const light &a, const light &b)
            { return a.p1.x < b.p1.x; });
        int left = 0, right = lights.size() - 1;

        while (left < right)
        {
            if (abs((lights[left].length / lights[right].length) - 1) < 0.2)
            {

                if (distance(lights[left].p1, lights[right].p1) > lights[left].length)
                {
                    array<Point, 4> points{lights[left].p1, lights[left].p2, lights[right].p2, lights[right].p1};
                    doors.push_back({lights[left], lights[right], (float)contourArea(points)});
                }
                ++left;
                --right;
            }
            else if (lights[left].length > lights[right].length)
            {
                ++left;
            }
            else
            {
                --right;
            }
        }
    }

    for (const light_pair &door : doors)
    {
        line(img, door.left.p1, door.right.p2, Scalar(0, 255, 255), 2);
        line(img, door.left.p2, door.right.p1, Scalar(0, 255, 255), 2);
    }

    return doors;
}   


void VisionNode::match_doors(vector<light_pair> left_doors, vector<light_pair> right_doors, vector<array<Point, 8>> &matched_points)
{
    sort(left_doors.begin(), left_doors.end(), [](const light_pair &a, const light_pair &b)
            { return a.area > b.area; });
    sort(right_doors.begin(), right_doors.end(), [](const light_pair &a, const light_pair &b)
            { return a.area > b.area; });

    int i = 0, j = 0;

    while (i < left_doors.size() && j < right_doors.size())
    {
        if (abs(left_doors[i].area / right_doors[j].area - 1) < 0.25)
        {
            const light_pair &l = left_doors[i];
            const light_pair &r = right_doors[j];

            matched_points.push_back({l.left.p1, l.right.p1, l.right.p2, l.left.p2, r.left.p1, r.right.p1, r.right.p2, r.left.p2});

            ++j; ++i;
        }
        else if (left_doors[i].area > right_doors[j].area)
        {
            ++i;
        }
        else
        {
            ++j;
        }
    }
}


Eigen::Vector3d VisionNode::compute_center_point(array<Point, 8> points) {
    array<Eigen::Vector3d, 4> points_3d;
    for (int i = 0; i < 4; ++i) {
        double d = points[i + 4].x - points[i].x; // 视差

        points_3d[i](0) = F * Base / d;
        points_3d[i](1) = Base * ((points[i + 4].x - Cx) / d + 0.5);
        points_3d[i](2) = - Base * (points[i + 4].y - Cy) / d;
    }
    return (points_3d[0] + points_3d[1] + points_3d[2] + points_3d[3]) / 4;
}


void VisionNode::img_callback(const sensor_msgs::ImageConstPtr &left_img_msg, const sensor_msgs::ImageConstPtr &right_img_msg) {
    if (!odom_init) return;
    try
    { 
        cv_bridge::CvImagePtr cv_ptr_l = cv_bridge::toCvCopy(left_img_msg, "bgr8");
        cv_bridge::CvImagePtr cv_ptr_r = cv_bridge::toCvCopy(right_img_msg, "bgr8");
        img_pre_process(cv_ptr_l->image, left_img);
        img_pre_process(cv_ptr_r->image, right_img);

        vector<array<Point, 8>> matched_points;
        match_doors(get_doors(right_img, cv_ptr_r->image), get_doors(left_img, cv_ptr_l->image), matched_points);


        for (const array<Point, 8> & points: matched_points) {

            Eigen::Vector3d local_point = compute_center_point(points);
            Eigen::Vector3d global_point = pose_R.inverse() * local_point + pose_t;

            cout << "global x: " << global_point[0] << " y: " << global_point[1] << " z: " << global_point[2] << endl; 
            cout << "pose_t x: " << pose_t[0] << " y: " << pose_t[1] << " z: " << pose_t[2] << endl; 
            cout << "c      x: " << local_point[0] << " y: " << local_point[1] << " z: " << local_point[2] << endl; 

            bool flag = true;
            // if (waypoints.empty()) flag = false;
            
            if (std::isnan(global_point[0]) || std::isnan(global_point[0]) || std::isnan(global_point[0])) continue;

            for (const auto &_ : waypoints){
                // cout << (distance(_, global_point)) << endl;
                if (distance(_, global_point) < 10 || distance(_, pose_t) < 10) {flag = false; break;};
            }

            if (flag) {
                waypoints.push_back(global_point);
                cout << "检测到新坐标点" << endl;
                cout << "global x: " << global_point[0] << " y: " << global_point[1] << " z: " << global_point[2] << endl;
                // send_waypoints();
            }

        }

        imshow("left", cv_ptr_l->image);
        imshow("right", cv_ptr_r->image);
        waitKey(1);

        // sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(left_img_msg->header, "mono8", left_img).toImageMsg();
        // pub_left_blur.publish(out_msg);        
        // sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(right_img_msg->header, "mono8", right_img).toImageMsg();
        // pub_right_blur.publish(out_msg);
    }
    catch (const cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception in right callback: %s", e.what());
    }
}


void VisionNode::yolo_cb(const yolov11_ros_msgs::BoundingBoxesConstPtr &msg)
{
    cout << endl;
    for (const yolov11_ros_msgs::BoundingBox &box : msg->bounding_boxes)
    {   
        cout << "class: "<< box.Class << endl;
        cout << "x  max min" << box.Class << endl;
        cout << "" << endl;
        cout << "x  " << box.xmax << " " << box.xmin << endl;
        cout << "y  " << box.ymax << " " << box.ymin << endl
                << endl;
        cout << "[" << box.ymax - box.ymin << "x" << box.xmax - box.xmin << "]" << endl
                << endl;
    }
}


void VisionNode::odom_cb(const nav_msgs::OdometryConstPtr pMsg) {
    odom_msg = *pMsg;
    pose_q = Eigen::Quaterniond(
        pMsg->pose.pose.orientation.w,
        pMsg->pose.pose.orientation.x,
        pMsg->pose.pose.orientation.y,
        pMsg->pose.pose.orientation.z
    );
    pose_R = pose_q.toRotationMatrix();
    pose_t = Eigen::Vector3d(
        pMsg->pose.pose.position.x,
        pMsg->pose.pose.position.y,
        pMsg->pose.pose.position.z
    );

    
    if (odom_init) {
        if (waypoints.size() > 0) {
            count += 1;
            if (distance(pose_t, waypoints.front()) < 5){
                waypoints.pop_front();
                cout << "抵达坐标点" << endl;
            }
        }
        return;
    }
    odom_init = true;
}
void VisionNode::send_waypoints(){
    path_sender::WayPoints tmp_points;
    cout << endl << endl << endl;
    for (const auto& _ : waypoints) {
        geometry_msgs::Point tmp_point;
        tmp_point.x = _[0];
        tmp_point.y = _[1];
        tmp_point.z = _[2];
        tmp_points.points.push_back(tmp_point);

        cout << "global x: " << _[0] << " y: " << _[1] << " z: " << _[2] << endl; 

    }
    waypoint_pub.publish(tmp_points);
}

void VisionNode::time_cb(const ros::TimerEvent &event)
{
    if (!waypoints.empty())
    {
        send_waypoints();
    }
}


VisionNode::VisionNode(ros::NodeHandle *nh) {
    std::cout << "OpenCV Version: " << cv::getVersionString() << std::endl;
    pub_right_blur = nh->advertise<sensor_msgs::Image>("/blur/front_right", 1);
    pub_left_blur = nh->advertise<sensor_msgs::Image>("/blur/front_left", 1);

    // ros::Subscriber sub_right = nh.subscribe<sensor_msgs::Image>("/airsim_node/drone_1/front_right/Scene", 1, rightImageCallback);
    // ros::Subscriber sub_left = nh.subscribe<sensor_msgs::Image>("/airsim_node/drone_1/front_left/Scene", 1, leftImageCallback);
    
    ros::Subscriber sub_yolo = nh->subscribe<yolov11_ros_msgs::BoundingBoxes>("/yolo/BoundingBoxes", 1, std::bind(&VisionNode::yolo_cb, this, std::placeholders::_1));
    ros::Subscriber sub_odom = nh->subscribe<nav_msgs::Odometry>("/Odometry", 2, std::bind(&VisionNode::odom_cb, this, placeholders::_1));

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> sub_left(*nh, "/airsim_node/drone_1/front_left/Scene", 2);
    message_filters::Subscriber<sensor_msgs::Image> sub_right(*nh, "/airsim_node/drone_1/front_right/Scene", 2);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_left, sub_right);
    sync.registerCallback(std::bind(&VisionNode::img_callback, this, placeholders::_1,placeholders::_2));

    waypoint_pub = nh->advertise<path_sender::WayPoints>("/waypoints", 1);
    timer = nh->createTimer(ros::Duration(2.0), &VisionNode::time_cb,this);

    

    ros::spin();

}

VisionNode::~VisionNode(){}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_test");
    ros::NodeHandle nh;
    ros::Duration(3.0).sleep();
    VisionNode vision_node(&nh);
    return 0;
}

