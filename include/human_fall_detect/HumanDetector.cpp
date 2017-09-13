#include "HumanDetector.h"

HumanDetector::HumanDetector()
{
    
}

HumanDetector::~HumanDetector()
{
    
}

void HumanDetector::ImportFromCvMat(cv::Mat img_dist)
{
    m_cloudScene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    m_cloudScene->width  = img_dist.cols;
    m_cloudScene->height = img_dist.rows;
    m_cloudScene->is_dense = false;
    
    m_cloudScene->points.resize (m_cloudScene->width * m_cloudScene->height);

    float x,y,z;
    for (int v = 0; v < img_dist.rows; v++) {
        for (int u = 0; u < img_dist.cols; u++) {
            z = img_dist.at<float>(v, u);
            x = (u - KINECT_CX) / KINECT_FX * z;
            y = (v - KINECT_CY) / KINECT_FY * z;
            m_cloudScene->points[u + v*img_dist.cols].x = x;
            m_cloudScene->points[u + v*img_dist.cols].y = y;
            m_cloudScene->points[u + v*img_dist.cols].z = z;
        }
    }

}

std::vector<int> HumanDetector::ExtractFloorFromScene()
{
    std::vector<int> res;
    res.reserve(m_cloudScene->points.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_grid (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (m_cloudScene);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*cloud_grid);
    std::cout << "cloud_grid->points.size(): " << cloud_grid->points.size() << std::endl;
    
    // Create the filtering object
//     pcl::PointIndicesPtr ground (new pcl::PointIndices);
//     pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
//     pmf.setInputCloud (cloud_grid);
//     pmf.setMaxWindowSize (20);
//     pmf.setSlope (1.0f);
//     pmf.setInitialDistance (0.5f);
//     pmf.setMaxDistance (5.0f);
//     pmf.extract (ground->indices);

    // Create the filtering object
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud (cloud_grid);
//     extract.setIndices (ground);
//     extract.filter (*cloud_filtered);
//     std::cout << "cloud_filtered->points.size(): " << cloud_filtered->points.size() << std::endl;   
    return res;
}