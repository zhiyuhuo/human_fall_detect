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
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define DEPTH_IMG_FACTOR 0.5
#define KINECT_FX (519*DEPTH_IMG_FACTOR)
#define KINECT_FY KINECT_FX
#define KINECT_CX (640*DEPTH_IMG_FACTOR/2)
#define KINECT_CY (480*DEPTH_IMG_FACTOR/2)


class HumanDetector {
    
public:
    HumanDetector();
    ~HumanDetector();
    
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudScene;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudVoxel;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudTranform;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudObjects;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudPassthrough;
    
public:
    void ImportFromCvMat(cv::Mat img_dist);
    bool VoxelizePoints(float voxel_size);
    bool TransformPointCloud();
    bool PassthroughPointCloud();
    
    int  GetMainPlane();
    
};

