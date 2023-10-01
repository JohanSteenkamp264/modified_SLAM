#include "Extractors/KAZEextractor.h"


#include <opencv2/features2d.hpp>
namespace ORB_SLAM3
{
using namespace cv;
using namespace std;
using cv::Ptr;
using cv::KAZE;

bool KAZEModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold)
{
    cv::Mat tDescriptors;
    Ptr<Feature2D> KAZE = KAZE::create(true, false, threshold, 1);
    KAZE->detectAndCompute(image, Mat(), vKeyPoints, tDescriptors);

    for(int id_kp = 0; id_kp < vKeyPoints.size(); id_kp++){
        vKeyPoints[id_kp].octave = 0.0;
        vKeyPoints[id_kp].size = 31.0;
    }

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
    Ptr<Feature2D> KAZE = KAZE::create(true, false, threshold, 1);
    KAZE->detectAndCompute(image, Mat(), vKeyPoints, tDescriptors);

    for(int id_kp = 0; id_kp < vKeyPoints.size(); id_kp++){
        vKeyPoints[id_kp].octave = 0.0;
        vKeyPoints[id_kp].size = 31.0;
    }

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
