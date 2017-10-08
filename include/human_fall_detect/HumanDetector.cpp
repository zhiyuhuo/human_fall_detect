#include "HumanDetector.h"

HumanDetector::HumanDetector()
{
    m_cloudScene       =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);   
    m_cloudVoxel       =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudTranform    =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudObjects     =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudPassthrough =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloud            =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    mHumanPos.x = 0;
    mHumanPos.y = 0;
    mHumanPos.z = 0;
}

HumanDetector::~HumanDetector()
{
    
}

void HumanDetector::ImportFromCvMat(cv::Mat img_dist)
{
    
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

bool HumanDetector::VoxelizePoints(float voxel_size)
{
    if (m_cloudScene->points.size() <= 0) {
        return false;
    }
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (m_cloudScene);
    vg.setLeafSize (voxel_size, voxel_size, voxel_size);
    vg.filter (*m_cloudVoxel);
    
    return true;
}

bool HumanDetector::TransformPointCloud()
{
    if (m_cloudVoxel->points.size() <= 0) {
        return false;
    }
    
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    float theta_z = M_PI;
    transform_1 (0,0) = cos (theta_z);
    transform_1 (0,1) = -sin(theta_z);
    transform_1 (1,0) = sin (theta_z);
    transform_1 (1,1) = cos (theta_z);
    
//     Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
//     float theta_y = M_PI;
//     transform_2 (0,0) = cos (theta_y);
//     transform_2 (0,1) = -sin(theta_y);
//     transform_2 (2,0) = sin (theta_y);
//     transform_2 (2,2) = cos (theta_y);
    
    Eigen::Matrix4f transform_3 = Eigen::Matrix4f::Identity();
    float theta_x = M_PI / 10;
    transform_3 (1,1) = cos (theta_x);
    transform_3 (1,2) = -sin(theta_x);
    transform_3 (2,1) = sin (theta_x);
    transform_3 (2,2) = cos (theta_x);
    
    Eigen::Matrix4f transform_4 = Eigen::Matrix4f::Identity();
    transform_4 (1,1) = 0;
    transform_4 (2,2) = 0;
    transform_4 (1,2) = 1;
    transform_4 (2,1) = 1;
    
    Eigen::Matrix4f transform_5 = Eigen::Matrix4f::Identity();
    float offset_z = 2.4;
    transform_5 (0,3) = 0.0;
    transform_5 (1,3) = 0.0;
    transform_5 (2,3) = offset_z;
    
    Eigen::Matrix4f transform = transform_5 
                                * transform_4
                                * transform_3 
//                                 * transform_2 
                                * transform_1;
    
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*m_cloudVoxel, *m_cloudTranform, transform);
    
    return true;
}

bool HumanDetector::PassthroughPointCloud()
{
    if (m_cloudTranform->points.size() <= 0) {
        return false;
    }
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (m_cloudTranform);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.10, 2.0);
    pass.filter (*m_cloudPassthrough);
    pass.setInputCloud (m_cloudPassthrough);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (2.0, 6.0);
    pass.filter (*m_cloudPassthrough);
    pass.setInputCloud (m_cloudPassthrough);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-3.0, 3.0);
    pass.filter (*m_cloudPassthrough);
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (m_cloudPassthrough);
    sor.setMeanK (50);
    sor.setStddevMulThresh (0.20);
    sor.filter (*m_cloudPassthrough);
    
    m_cloud = m_cloudPassthrough;
    return true;
}

int HumanDetector::GetMainPlane()
{
    if (m_cloudVoxel->points.size() <= 0) {
        return -1;
    }
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.10);

    seg.setInputCloud (m_cloudVoxel);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    std::cerr << "m_cloudVoxel->points.size(): " << m_cloudVoxel->points.size() << std::endl;
    
    return inliers->indices.size();
}

int HumanDetector::ExtractAndTrackHumanTarget()
{
    
    
}
