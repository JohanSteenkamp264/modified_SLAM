#include "Extractors/SIFTextractor.h"


#include <opencv2/features2d.hpp>

using namespace std;

namespace ORB_SLAM3
{

SIFTModel::SIFTModel(int _nfeatures, int _nOctaveLayers, double _contrastThreshold, double _edgeThreshold, double _sigma)
:nfeatures(_nfeatures), nOctaveLayers(_nOctaveLayers), contrastThreshold(_contrastThreshold), edgeThreshold(_edgeThreshold), sigma(_sigma)
{
    sift = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
}

bool SIFTModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold)
{
    //cout << "Extracting features" << endl;
    cv::Mat tDescriptors;
    sift->detectAndCompute(image, cv::Mat(), vKeyPoints, tDescriptors);

    int octave = 0;
    int layer = 0;

    for(int id_kp = 0; id_kp < vKeyPoints.size(); id_kp++){
        octave = (vKeyPoints[id_kp].octave + 1)&255;
        layer = (vKeyPoints[id_kp].octave >> 8) & 255;

        //cout << octave << " " << layer << endl;

        octave = octave < 128 ? octave : (-128 | octave);  

        vKeyPoints[id_kp].octave = octave;
        //vKeyPoints[id_kp].size = 16.0*octave;
        //cout << vKeyPoints[id_kp].octave << " " << octave << " " << layer << endl;
    }
    tDescriptors.copyTo(localDescriptors);
    //cout << "Extracting " << vKeyPoints.size() <<  " features" << endl;

    globalDescriptors = cv::Mat(4096, 1, CV_32F);
    float* vResGlobalDescriptor = tDescriptors.ptr<float>(0);
    for (int temp = 0; temp < 4096; ++temp)
    {
        globalDescriptors.ptr<float>(0)[temp] = vResGlobalDescriptor[temp];
    }
    return true;
}

bool SIFTModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold)
{
    cv::Mat tDescriptors;
    sift->detectAndCompute(image, cv::Mat(), vKeyPoints, tDescriptors);

    int octave = 0;
    int layer = 0;

    for(int id_kp = 0; id_kp < vKeyPoints.size(); id_kp++){
        octave = (vKeyPoints[id_kp].octave + 1)&255;
        layer = (vKeyPoints[id_kp].octave >> 8) & 255;

        //cout << octave << " " << layer << endl;

        octave = octave < 128 ? octave : (-128 | octave);  

        vKeyPoints[id_kp].octave = (octave*3) + layer - 1;
        vKeyPoints[id_kp].size = 16.0*octave;
        //cout << vKeyPoints[id_kp].octave << " " << octave << " " << layer << endl;
    }
    tDescriptors.copyTo(localDescriptors);

    cout << "Extracting " << vKeyPoints.size() <<  " features" << endl;

    return true;
}

bool SIFTModel::Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors)
{
    intermediate.copyTo(globalDescriptors);
    return false;
}

} // ORB_SLAM3
