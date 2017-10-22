#include "HumanDetector.h"

HumanDetector::HumanDetector()
{
    // point clouds
    m_cloudScene       =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudVoxel       =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudTranform    =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudObjects     =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudPassthrough =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloud            =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloudHuman       =    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // human centroid
    mHumanPos.x = 0;
    mHumanPos.y = 0;
    mHumanPos.z = 0;
    
    // probability grid map
    mGridMap = cv::Mat::zeros(100, 100, CV_32FC1);
    
    // frame counter
    mCountFrame = 0;
    mIfPreprocessReady = false;
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
    sor.setStddevMulThresh (0.10);
    sor.filter (*m_cloudPassthrough);
    
    return true;
}

bool HumanDetector::PassthroughHumanTarget()
{
    bool res = false;
    m_cloudHuman->clear();
    
    // remove the points in the occupied regions
    if (mIfPreprocessReady) {
        for (int i = 0; i < m_cloudPassthrough->points.size(); i++) {
            int xx = int(m_cloudPassthrough->points[i].x / 0.1 + 0.5) + mGridMap.cols/2;
            int yy = int(m_cloudPassthrough->points[i].y / 0.1 + 0.5);
            
            if (mGridMap.at<float>(yy,xx) == 0) {
                m_cloudHuman->push_back(m_cloudPassthrough->points[i]);
            }
        }
    }
    
    // find the centroid of the current point cloud;
    float cx = 0;
    float cy = 0;
    for (int i = 0; i < m_cloudHuman->points.size(); i++) {
        cx += m_cloudHuman->points[i].x;
        cy += m_cloudHuman->points[i].y;
    }
    
    cx /= m_cloudHuman->size();
    cy /= m_cloudHuman->size();
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (m_cloudHuman);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (cy-0.5, cy+0.5);
    pass.filter (*m_cloudHuman);
    pass.setInputCloud (m_cloudHuman);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (cx-0.5, cx+0.5);
    pass.filter (*m_cloudHuman);
    
    return res;
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
    VoxelizePoints(0.05);
    TransformPointCloud();
    PassthroughPointCloud();
    PassthroughHumanTarget();
    
    m_cloud = m_cloudHuman;
}

int HumanDetector::LearnGridMap()
{
    const int width = mGridMap.cols;
    const int height = mGridMap.rows;
    cv::Mat checkedMap = cv::Mat::zeros(height, width, CV_8UC1);
    const int step = width;
    for (int i = 0; i < m_cloudPassthrough->points.size(); i++) {
        int xx = int(m_cloudPassthrough->points[i].x / 0.1 + 0.5) + mGridMap.cols/2;
        int yy = int(m_cloudPassthrough->points[i].y / 0.1 + 0.5);
        
        if (xx >=0 && xx < width && yy >=0 && yy < height) {
            checkedMap.data[xx + yy * step] = 1;
        }
    }
    
    float* gridMapData = mGridMap.ptr<float>(0);
    for (int i = 0; i < mGridMap.cols * mGridMap.rows; i++) {
        if (checkedMap.data[i]) {
            gridMapData[i]++;
        }
    }
    
    return 0;
}

int HumanDetector::NormalizeGridMap()
{
    double minValue, maxValue;
    cv::minMaxLoc(mGridMap, &minValue, &maxValue);
    mGridMap = mGridMap / maxValue;
    
    float* gridMapData = mGridMap.ptr<float>(0);
    for (int i = 0; i < mGridMap.cols * mGridMap.rows; i++) {
        if (gridMapData[i] < 0.1) {
            gridMapData[i] = 0;
        } else 
        {
            gridMapData[i] = 1.0;
        }
    }
    
    cv::imshow("grid map", mGridMap);
}

int HumanDetector::Preprocess()
{
    if (mCountFrame < 30) { 
        LearnGridMap();
    }
    else if (mCountFrame == 30) {
        NormalizeGridMap();
        mIfPreprocessReady = true;
    }
    mCountFrame++;
}


