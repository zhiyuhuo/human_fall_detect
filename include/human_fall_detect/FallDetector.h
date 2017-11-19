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
    ObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float time_stamp);
    ~ObjectFrame();
    
public: // raw data
    std::vector<cv::Point3f> points;
    float timeStamp;
    
public: // features
    float fZmax;
    float fZcent;
    float fZpg;
    
    float fVs;
};

class FallDetector {
    
public:
    FallDetector();
    ~FallDetector();
    
public:
    std::vector<ObjectFrame> m_objectFrameSet;
    std::vector<float> mKeyframes;
    std::vector<float> mVsSet;
    std::vector<float> mTsSet;
    
public:
    void  AddObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float time_stamp);
    void  ProcessObjectFrame();
    
    // 1st Stage: Vertical State Characterization
    float GetMaxHeight(std::vector<cv::Point3f>& points);
    float GetCentroidHeight(std::vector<cv::Point3f>& points);
    float GetProjectGround(std::vector<cv::Point3f>& points);
    
    float GetVerticalState(std::vector<cv::Point3f>& points);
    void  FilterVector(std::vector<float>& data, int win_size);
    std::vector<float>  GetEventSegmentation(std::vector<float>& Vs_set, std::vector<float>& Ts_set, int f_rate);
    int   AnalyzeFallEvent();
    
    // 2nd Stage: On Ground Event Features
    float GetMinVerticalVelocity(float& timeStampMinVV);
    float GetMaxVerticalAcceleration(float timeStampMinVV) ;
    float GetMeanVavg();
    float GetOcclusionAdjustedChangeInZpg();
    float GetMinimumFrameToFrameVerticalVelocity();
    
    float GetFallConfidence();
    
    // functions
    cv::Point3f GetCentroidofPoints(std::vector<cv::Point3f>& points);
    void  TransformPointCloud(cv::Mat R, cv::Mat t, std::vector<cv::Point3f>& points);

};

#endif

