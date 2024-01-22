#include "Extractors/SURFextractor.h"


using namespace std;

namespace ORB_SLAM3
{
SURFModel::SURFModel(double _hessianThreshold, int _nOctaves, int _nOctaveLayers, bool _extended, bool _upright)
:hessianThreshold(_hessianThreshold), nOctaves(_nOctaves), nOctaveLayers(_nOctaveLayers), extended(_extended), upright(_upright)
{
    surf = cv::xfeatures2d::SURF::create(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
}


bool SURFModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold)
{
    cv::Mat tDescriptors;
    surf->detectAndCompute(image, cv::Mat(), vKeyPoints, tDescriptors);
    
    if(vKeyPoints.size() > nKeypointsNum)
    {
        tDescriptors = tDescriptors.rowRange(0,nKeypointsNum);
	vKeyPoints.resize(nKeypointsNum);
    };

    tDescriptors.copyTo(localDescriptors);

    globalDescriptors = cv::Mat(4096, 1, CV_32F);
    float* vResGlobalDescriptor = tDescriptors.ptr<float>(0);
    for (int temp = 0; temp < 4096; ++temp)
    {
        globalDescriptors.ptr<float>(0)[temp] = vResGlobalDescriptor[temp];
    }
    return true;
}

bool SURFModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold)
{
    cv::Mat tDescriptors;
    surf->detectAndCompute(image, cv::Mat(), vKeyPoints, tDescriptors);

    if(vKeyPoints.size() > nKeypointsNum)
    {
        tDescriptors = tDescriptors.rowRange(0,nKeypointsNum);
	    vKeyPoints.resize(nKeypointsNum);
    }

    tDescriptors.copyTo(localDescriptors);
    return true;
}

bool SURFModel::Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors)
{
    intermediate.copyTo(globalDescriptors);
}

} // ORB_SLAM3
