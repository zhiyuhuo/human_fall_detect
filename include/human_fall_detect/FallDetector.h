#ifndef FEATUREEXTRACTOR_H_
#define FEATUREEXTRACTOR_H_

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/voxel_grid.h>

class ObjectFrame {
public:
    ObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double time_stamp);
    ~ObjectFrame();
    
public:
    std::vector<cv::Point3f> points;
    double timeStamp;
};

class FallDetector {
    
public:
    FallDetector();
    ~FallDetector();
    
public:
    std::vector<ObjectFrame> m_objectFrameSet;
    
public:
    void AddObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double time_stamp);
    
    // 1st Stage: Vertical State Characterization
    float GetMaxHeight(const std::vector<cv::Point3f>& points);
    float GetCentroidHeight(const std::vector<cv::Point3f>& points);
    float GetProjectGround(const std::vector<cv::Point3f>& points);
    
    float GetVerticalState();
    void  FilterObjectFrames();
    void  GetEventTimeFromObjectFrames();
    
    // 2nd Stage: On Ground Event Features
    float GetMinVerticalVelocity();
    float GetMaxVerticalVelocity();
    float GetMeanVavg();
    float GetOcclusionAdjustedChangeInZpg();
    float GetMinimumFrameToFrameVerticalVelocity();
    
    float GetFallConfidence();
    
    // functions
    void TransformPointCloud(cv::Mat R, cv::Mat t, std::vector<cv::Point3f>& points);
    void OpenCVPointCloudViewer(cv::Mat& img, std::vector<cv::Point3f> points, std::vector<cv::Vec3b> colors);
};

#endif

