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

float FallDetector::GetMaxHeight(const std::vector<cv::Point3f>& points, int& maxheight_index)
{
    float max_height = -1.0; 
    for (int i = 0; i < points.size(); i++) {
        if (points[i].z > max_height) {
            max_height = points[i].z;
        }
    }
    
    return max_height;
}

float FallDetector::GetCentroidHeight(const std::vector<cv::Point3f>& points)
{
    float res = 0;
    cv::Point3f centroid = GetCentroidofPoints(points);
    
    res = centroid.z;
    
    return res;
}

float FallDetector::GetProjectGround(const std::vector<cv::Point3f>& points)
{
    float res = 0;
    cv::Point3f centroid = GetCentroidofPoints(points);
    std::vector<cv::Point3f> points_local = points;
    float cell_area = 0.0254*0.0254;
    for (int i = 0; i < points.size(); i++) {
        points_local.x -= centroid.x;
        points_local.y -= centroid.y;
    }
    
    float height_th = 0.38;
    for (int i = 0; i < points_local.size(); i++) {
        if (points_local.z > height_th) {
            int idx = points_local.x / 0.0254 + 10000 * points_local.y;
        }
    }
    
    std::set<int> cells_list(0);
    
    return res;
}

void FallDetector::AddObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double time_stamp)
{
    m_objectFrameSet.push_back(ObjectFrame(cloud, time_stamp));
}

cv::Point3f FallDetector::GetCentroidofPoints(std::vector<cv::Point3f>& points)
{
    cv::Point3f res;
    int   N   = points.size();
    
    for (int i = 0; i < N; i++) {
        res = res + points[i];
    }
    
    res = res * (1.0 / N);
    
    return res;
}

void FallDetector::TransformPointCloud(cv::Mat R, cv::Mat t, std::vector<cv::Point3f>& points)
{
    float* R_ = (float*)(&R.at<float>(0));
    float* t_ = (float*)(&t.at<float>(0));
    cv::Point3f pt;
    for (int i = 0; i < points.size(); i++) {
        pt = points[i];
        points[i].x = R_[0]*pt.x + R_[1]*pt.y + R_[2]*pt.z + t_[0];
        points[i].y = R_[3]*pt.x + R_[4]*pt.y + R_[5]*pt.z + t_[1];
        points[i].z = R_[6]*pt.x + R_[6]*pt.y + R_[8]*pt.z + t_[2];
    }
}

void FallDetector::OpenCVPointCloudViewer(cv::Mat& img, std::vector<cv::Point3f> points, std::vector<int> index)
{
    
}