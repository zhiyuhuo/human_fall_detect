#include "FallDetector.h"

ObjectFrame::ObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float time_stamp)
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

void FallDetector::AddObjectFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float time_stamp)
{
    m_objectFrameSet.push_back(ObjectFrame(cloud, time_stamp));
}

void FallDetector::ProcessObjectFrame()
{
    // get the vertical state of each frame and save it to m_objectFrameSet.
    float verticalState = GetVerticalState(m_objectFrameSet.back().points);
    std::cout << "vertical state: " << verticalState << ", " << m_objectFrameSet.back().timeStamp << std::endl;
    m_objectFrameSet.back().fVs = verticalState;

    // Get the key frames of a fall.
    if (m_objectFrameSet.back().timeStamp > 190) {
        AnalyzeFallEvent();
    }
}

float FallDetector::GetMaxHeight(std::vector<cv::Point3f>& points)
{
    float max_height = -1.0; 
    for (int i = 0; i < points.size(); i++) {
        if (points[i].z > max_height) {
            max_height = points[i].z;
        }
    }
    
    return max_height;
}

float FallDetector::GetCentroidHeight(std::vector<cv::Point3f>& points)
{
    float res = 0;
    cv::Point3f centroid = GetCentroidofPoints(points);
    
    res = centroid.z;
    
    return res;
}

float FallDetector::GetProjectGround(std::vector<cv::Point3f>& points)
{
    float res = 0;
    cv::Point3f centroid = GetCentroidofPoints(points);
    std::vector<cv::Point3f> points_local = points;
    for (int i = 0; i < points.size(); i++) {
        points_local[i].x -= centroid.x;
        points_local[i].y -= centroid.y;
    }
    
    std::set<int> cells;
    float height_th = 0.38;
    for (int i = 0; i < points_local.size(); i++) {
        if (points_local[i].x > -1 && points_local[i].x < 1
            && points_local[i].x > -1 && points_local[i].x < 1) {
            if (points_local[i].z < height_th) {
                cells.insert(int(points_local[i].x+0.5) + 20 * int(points_local[i].y+0.5));
            }
        }
    }
    
    res = cells.size();
    return res;
}

float FallDetector::GetVerticalState(std::vector<cv::Point3f>& points)
{
    float Zmax = GetMaxHeight(points);
    float Zcent = GetCentroidHeight(points);
    float Zpg = GetProjectGround(points);
    
    float Kmax = 1.70;
    float Kcent = 0.85; // kmax/2
    float Kpg = 23.8;
    
    float Vs = (Zmax/Kmax + Zcent/Kcent + Zpg/Kpg);
    return Vs;
}

void FallDetector::FilterVector(std::vector<float>& data, int win_size)
{
    
}

std::vector<float> FallDetector::GetEventSegmentation(std::vector<float>& Vs_set, std::vector<float>& Ts_set, int f_rate)
{
    std::vector<float> res(4, 0);
    
    int N = Vs_set.size();
    int i_start;
    int i_init;
    int i_end;
    int i_fall;
    
    int i = 0;
    float Ttrig = 0.885;
    float epsl = 0.05;

    while (i < N-1) {
        if (Vs_set[i] < Ttrig) {
            i_start = i_init = i;
            
            while (i_start < N && Vs_set[i_start+1] < Vs_set[i_start]-epsl) {
                i_start++;
            }
            
            i_fall = i_start;
            
            while (i_fall > std::max(0, i_start - 4*f_rate) && Vs_set[i_fall-1] > Vs_set[i_fall]) {
                i_fall = i_fall - 1;
            }
            
            i_end = i_start;
            
            while (i_end < N-1 && Vs_set[i_end] < Ttrig) {
                i_end++;
            }
            
            i = i_end + 1;
        } else {
            i++;
        }
    }
    
    res[0] = Ts_set[i_fall];
    res[1] = Ts_set[i_init];
    res[2] = Ts_set[i_start];
    res[3] = Ts_set[i_end];
    
    return res;
}

int FallDetector::AnalyzeFallEvent()
{
    std::vector<float> vsSet;
    std::vector<float> tsSet;
    float fRate = 1;

    int StartFrame = 130;
    int EndFrame   = 180;
    for (int i = StartFrame; i < EndFrame; i++) {
        vsSet.push_back(m_objectFrameSet[i].fVs);
        tsSet.push_back(m_objectFrameSet[i].timeStamp);
    }

    std::vector<float> fallTime = GetEventSegmentation(vsSet, tsSet, fRate);
    std::cout << "fall: " << fallTime[0] 
              << " init: " << fallTime[1]
              << " start: " << fallTime[2]
              << " fend: " << fallTime[3] << std::endl; 

    mVsSet     = vsSet;
    mTsSet     = tsSet;
    mKeyframes = fallTime;
    return 1;
}

float FallDetector::GetMinVerticalVelocity(float& timeStampMinVV)
{
    float res;
    
    float tfall  = mKeyframes[0];
    float tstart = mKeyframes[1];
    float minVV = 999999.;
    float tMinVV = 0;
    for (int i = 0; i < mVsSet.size(); i++) {
        if (mTsSet[i] >= tfall && mTsSet[i] <= tstart) {
            float VV = fabs((mVsSet[i+1] - mVsSet[i-1]) / 2);
            if (VV < minVV) {
                minVV = VV;
                tMinVV = mTsSet[i];
            }
        }
    }
    res = minVV;
    timeStampMinVV = tMinVV;

    return res;
}

float FallDetector::GetMaxVerticalAcceleration(float timeStampMinVV) 
{
    float res;
    
    float tstart = mKeyframes[1];
    float maxVA = 0.;
    for (int i = 0; i < mVsSet.size(); i++) {
        if (mTsSet[i] >= timeStampMinVV && mTsSet[i] <= tstart) {
            float VA = fabs((mVsSet[i+1] + mVsSet[i-1] - 2*mVsSet[i]) / 2);
            if (VA > maxVA) {
                maxVA = VA;
            }
        }
    }
    res = maxVA;

    return res;
}

float FallDetector::GetMeanVavg()
{
    float res;
    
    float tstart = mKeyframes[1];
    float tfend  = mKeyframes[3];
    float meanVV = 0;
    for (int i = 0; i < mVsSet.size(); i++) {
        if (mTsSet[i] > tstart && mTsSet[i] <= tfend) {
            meanVV += mVsSet[i];
        }
    }
    res = meanVV / (tfend - tstart);

    return res;

}

float FallDetector::GetOcclusionAdjustedChangeInZpg()
{
    float res;

    return res;
}

float FallDetector::GetMinimumFrameToFrameVerticalVelocity()
{
    float res;

    return res;
}

float FallDetector::GetFallConfidence()
{
    float timeStampMinVV;
    float minVV = GetMinVerticalVelocity(timeStampMinVV);
    float maxVA = GetMaxVerticalAcceleration(timeStampMinVV);
    float meanV = GetMeanVavg();
    float OACZ  = GetOcclusionAdjustedChangeInZpg();
    float MFFVV = GetMinimumFrameToFrameVerticalVelocity();
}

cv::Point3f FallDetector::GetCentroidofPoints(std::vector<cv::Point3f>& points)
{
    cv::Point3f res;
    int   N   = points.size();
    
    for (int i = 0; i < N; i++) {
        res = res + points[i];
    }
    
    if (N > 0) {
        res = res * (1.0 / N);
    } else {
        res = cv::Point3f(0, 0, 0);
    }
    
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

