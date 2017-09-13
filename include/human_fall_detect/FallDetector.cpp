#include "FallDetector.h"

ObjectFrame::ObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double time_stamp)
{
    points.clear();
    for (int i = 0; i < cloud->points.size(); i++) {
        points.push_back(cv::Point3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }
    timeStamp = time_stamp;
}

ObjectFrame::~ObjectFrame()
{

}

FallDetector::FallDetector()
{
    
}

FallDetector::~FallDetector()
{
    
}

void FallDetector::AddObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double time_stamp)
{
    m_objectFrameSet.push_back(ObjectFrame(cloud, time_stamp));
}