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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

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
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

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

    ros::init(argc, argv, "visualize_pc");
    ros::NodeHandle nh;

    ros::Subscriber sub_depth = nh.subscribe("/camera/depth/image_raw", 1, &depthCallback);

    ros::Rate rate(30);
    HumanDetector human_detector;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer;
    pclViewer = simpleVis(human_detector.m_cloudPassthrough);
    
    bool if_set_points = false;;
    while (ros::ok() 
           && !pclViewer->wasStopped ())
    {
        if ( if_get_depth ) {
            
            cv::pyrDown(img_depth, img_down, cv::Size(img_down.cols, img_down.rows));
            img_down.convertTo(img_dist, CV_32FC1, 1/1000.0);
            cv::Mat img_show = img_dist.clone() / 6.0;
            cv::imshow("dist", img_show);
            cv::waitKey(1);

            human_detector.ImportFromCvMat(img_dist);
            human_detector.ExtractAndTrackHumanTarget();
            human_detector.Preprocess();
            
            pclViewer->updatePointCloud(human_detector.m_cloud, "sample cloud");
            
            pclViewer->spinOnce (30);
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

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition( 0, -3.0, 8.0, 0, 6.0, 0, 0, 0, 1 );
    return (viewer);
}

