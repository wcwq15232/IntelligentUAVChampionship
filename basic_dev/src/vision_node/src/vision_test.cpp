#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <yolov11_ros_msgs/BoundingBoxes.h>
#include <yolov11_ros_msgs/BoundingBox.h>

#include <vector>
#include <array>
#include <cmath>

using namespace cv;
using namespace std;

int thre = 158;
int rate1 = 100;
int rate2 = 100;

struct light
{
    Point p1;  // 高
    Point p2;  // 低
};

struct light_pair
{
    light left;
    light right;
};


ros::Publisher pub_right_blur;
ros::Publisher pub_left_blur;

Mat kernel_7 = getStructuringElement(MORPH_RECT, Size(7, 7));
Mat kernel_5 = getStructuringElement(MORPH_RECT, Size(5, 5));
Mat kernel_3 = getStructuringElement(MORPH_RECT, Size(3, 3));

bool getContours(Mat& imgDil, Mat& img){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // drawContours(img, contours, -1, Scalar(255, 0, 255), 2);
    bool result = false;

    vector<double> areas(contours.size());

    int max_1 = -1, max_2 = -1;
    int max_1_ind, max_2_ind;


    for (int i = 0; i < contours.size(); ++i){
        areas[i] = contourArea(contours[i]);
        // if (areas[i] < 100) continue;

        // if (areas[i] > max_1) {
        //     max_2 = max_1;
        //     max_2_ind = max_1_ind;
        //     max_1 = areas[i];
        //     max_1_ind = i;
        // } else if (areas[i] > max_2) {
        //     max_2 = areas[i];
        //     max_2_ind = i;
        // }
    }

    vector<vector<Point>> conPoly(contours.size());
    vector<light> lights;

    for (int i = 0; i < contours.size(); ++i){
        // if (i != max_1_ind && i != max_2_ind) continue;
        if (areas[i] < 100) {
            continue;
        }

        result = true;
        float peri = arcLength(contours[i], true);
        approxPolyDP(contours[i], conPoly[i], 0.03 * peri, true);
        // drawContours(img, conPoly, i, Scalar(0, 0, 255), 2, LINE_4);
        cout << contours[i].size() << endl;
        cout << conPoly[i].size() << endl << endl;

        if (conPoly[i].size() == 2){

            
            double angle = abs(atan2(conPoly[i][0].x - conPoly[i][1].x, conPoly[i][0].y - conPoly[i][1].y) / M_PI);
            cout << angle << endl;
            putText(img, to_string(angle), (conPoly[i][0] + conPoly[i][1]) / 2, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 1);
            if (conPoly[i][0].y > conPoly[i][1].y){
                lights.push_back({conPoly[i][0], conPoly[i][1]});
            } else {
                lights.push_back({conPoly[i][1], conPoly[i][0]});
            }

            if (angle > 0.75) {
                line(img, conPoly[i][0], conPoly[i][1], Scalar(0, 255, 0), 2);
                circle(img, conPoly[i][0], 5, Scalar(255, 0, 0), 3);
                circle(img, conPoly[i][1], 5, Scalar(255, 0, 0), 3);
            }
            // if (i == max_1_ind || i == max_2_ind) {
            //     circle(img, conPoly[i][0], 5, Scalar(0, 255, 0), 2);
            //     circle(img, conPoly[i][1], 5, Scalar(0, 255, 0), 2);
            // }
        }
        // boundRect[i] = boundingRect(conPoly[i]);
        // rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2);

    }

    // sort(areas.begin(), areas.end());
    // for (double _: areas){
    //     cout << _ << "  ";
    // }
    // cout << endl;
    return result;
}

void compute_points(Mat& img, Mat& dst){
    vector<Mat> channels;
    split(img, channels);
    
    Mat RG = channels[2] - channels[1];
    // Mat RB = channels[2] - channels[0];
    // Mat BG = channels[0] - channels[1];
    // Mat BR = channels[0] - channels[2];
    // Mat GB = channels[1] - channels[0];
    // Mat GR = channels[1] - channels[2];
    Mat binary;
    Mat binary1;
    // Mat new_img = (BR + RB) * (rate1 / 100.0);
    // Mat new_img = channels[2] - (rate1 / 100.0) * channels[1] - (1 - (rate1 / 100.0)) * channels[0];
    Mat new_img = channels[2] - channels[1];
    threshold(new_img, binary, 28, 255, THRESH_BINARY);


    // Mat eroded;
    // erode(binary, eroded, kernel_3);
    dilate(binary, dst, kernel_3);
    bitwise_and(binary, dst, binary1);

    getContours(binary1, img);

    // imshow("src", img);
    // imshow("1", new_img);
    imshow("binary", binary1);
    imshow("dilated", img);

}

void rightImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        Mat dst;

        compute_points(cv_ptr->image, dst);


        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "mono8", dst).toImageMsg();
        pub_right_blur.publish(out_msg);
        waitKey(1);

    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception in right callback: %s", e.what());
    }
}

void leftImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        Mat blurred;
        bilateralFilter(cv_ptr->image, blurred, 9, 75, 75);

        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", blurred).toImageMsg();
        pub_left_blur.publish(out_msg);
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception in left callback: %s", e.what());
    }
}

void yolo_cb(const yolov11_ros_msgs::BoundingBoxesConstPtr& msg)
{   
    cout << endl;
    for (const yolov11_ros_msgs::BoundingBox& box: msg->bounding_boxes){
        cout << "x  max min" << box.Class << endl;
        cout << "" << endl;
        cout << "x  " << box.xmax << " " << box.xmin << endl;
        cout << "y  " << box.ymax << " " << box.ymin << endl << endl;
        cout << "[" << box.ymax - box.ymin << "x" <<  box.xmax - box.xmin  << "]" << endl << endl;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "img_test");
    ros::NodeHandle nh;

    std::cout << "OpenCV Version: " << cv::getVersionString() << std::endl;
    pub_right_blur = nh.advertise<sensor_msgs::Image>("/blur/front_right", 1);
    pub_left_blur = nh.advertise<sensor_msgs::Image>("/blur/front_left", 1);

    ros::Subscriber sub_right = nh.subscribe("/airsim_node/drone_1/front_right/Scene", 1, rightImageCallback);
    ros::Subscriber sub_left = nh.subscribe("/airsim_node/drone_1/front_left/Scene", 1, leftImageCallback);
    ros::Subscriber sub_yolo = nh.subscribe("/yolo/BoundingBoxes", 1, yolo_cb);

    namedWindow("T", WINDOW_NORMAL);  // "T"为窗口名
    resizeWindow("T", 640, 200);      // 设置窗口大小
    
    //                滑块名    父窗口  int指针 最大值  回调函数(可选)
    createTrackbar("threshold", "T",  &thre, 255);  
    createTrackbar("r1", "T",  &rate1, 100);  
    createTrackbar("r2", "T",  &rate2, 255);  
    waitKey(1);  

    ros::spin();

    return 0;
}