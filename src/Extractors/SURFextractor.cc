#include "Extractors/SURFextractor.h"


#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;

namespace ORB_SLAM3
{

bool SURFModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold)
{
    cv::Mat tDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(100, 1);
    surf->detectAndCompute(image, cv::Mat(), vKeyPoints, tDescriptors);

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

bool SURFModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold)
{
    cv::Mat tDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(100, 1);
    surf->detectAndCompute(image, cv::Mat(), vKeyPoints, tDescriptors);

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

bool SURFModel::Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors)
{
    intermediate.copyTo(globalDescriptors);
}

} // ORB_SLAM3
