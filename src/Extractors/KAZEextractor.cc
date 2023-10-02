#include "Extractors/KAZEextractor.h"

namespace ORB_SLAM3
{
using namespace cv;
using namespace std;
using cv::Ptr;
using cv::KAZE;


KAZEModel::KAZEModel(bool _extended, bool _upright, float _threshold, int _nOctaves, int _nOctaveLayers)
:extended(_extended), upright(_upright), threshold(_threshold), nOctaves(_nOctaves), nOctaveLayers(_nOctaveLayers)
{
    KAZE = KAZE::create(extended, upright, threshold, nOctaves, nOctaveLayers);
}

bool KAZEModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold)
{
    cv::Mat tDescriptors;
    KAZE->detectAndCompute(image, Mat(), vKeyPoints, tDescriptors);

    if(vKeyPoints.size() > nKeypointsNum)
    {
        tDescriptors = tDescriptors.rowRange(0,nKeypointsNum);
	    vKeyPoints.resize(nKeypointsNum);
    }

    tDescriptors.copyTo(localDescriptors);

	
    globalDescriptors = cv::Mat(4096, 1, CV_32F);
  
    float* vResGlobalDescriptor = tDescriptors.ptr<float>(0);
    for (int temp = 0; temp < 4096; ++temp)
    {
        globalDescriptors.ptr<float>(0)[temp] = vResGlobalDescriptor[temp];
    }

    
    return true;
}

bool KAZEModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold)
{
    cv::Mat tDescriptors;
    KAZE->detectAndCompute(image, Mat(), vKeyPoints, tDescriptors);


    if(vKeyPoints.size() > nKeypointsNum)
    {
        tDescriptors = tDescriptors.rowRange(0,nKeypointsNum);
	vKeyPoints.resize(nKeypointsNum);
    }

    tDescriptors.copyTo(localDescriptors);
    return true;
}

bool KAZEModel::Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors)
{
    intermediate.copyTo(globalDescriptors);
}

} // ORB_SLAM3
