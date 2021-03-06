#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "human_fall_detect/HumanDetector.h"

using namespace std;
using namespace cv;


const int IMGDEPTHWIDTH = 640;
const int IMGDEPTHHEIGHT = 480;
const int IMGDEPTHTYPE = CV_16UC1;
const int IMGDISTTYPE  = CV_32FC1;

Mat img_depth;
Mat img_down;
Mat img_dist;

void depthCallback(const sensor_msgs::ImageConstPtr& msg);
bool if_get_depth;
uint32_t depth_time_ns;


int main(int argc, char **argv)
{
    depth_time_ns = 0;

    // initialize data container
    img_depth = cv::Mat::zeros(IMGDEPTHHEIGHT, IMGDEPTHWIDTH, IMGDEPTHTYPE);
    img_down  = cv::Mat::zeros(IMGDEPTHHEIGHT/2, IMGDEPTHWIDTH/2, IMGDEPTHTYPE);
    img_dist  = cv::Mat::zeros(IMGDEPTHHEIGHT/2, IMGDEPTHWIDTH/2, IMGDISTTYPE);

    if_get_depth = false;

    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_depth = nh.subscribe("/camera/depth/image_raw", 1, &depthCallback);

    ros::Rate rate(30);
    while (ros::ok())
    {
        if ( if_get_depth ) {

            cv::pyrDown(img_depth, img_down, cv::Size(img_down.cols, img_down.rows));
            img_down.convertTo(img_dist, CV_32FC1, 1/1000.0);
            cv::imshow("dist", img_dist);
            HumanDetector human_detector;
            human_detector.ImportFromCvMat(img_dist);
            cv::waitKey(1);
        
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if_get_depth = true;
    depth_time_ns = msg->header.stamp.nsec;
    //depth_time_ns = msg->header.stamp.to_sec();
    //std::cout << "depth: " << msg->width << " " << msg->height << " " << msg->step << " " << msg->encoding << endl;
    std::memcpy(img_depth.data, (float*)&msg->data[0], msg->step*msg->height);
}

