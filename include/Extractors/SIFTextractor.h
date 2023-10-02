#ifndef SIFT_EXTRACTOR_H
#define SIFT_EXTRACTOR_H

#include <cmath>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Extractors/BaseModel.h"
namespace ORB_SLAM3
{

class SIFTModel: public BaseModel
{
public:
    SIFTModel(int nfeatures, int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma);
    ~SIFTModel(){};
    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors) override;

    bool IsValid(void) override { return true; };

    ModelType Type(void) override {return oCVSIFTModel;};

    bool getScaleValues(float &scaleFactor, int &nLevels, std::vector<float> & mvScaleFactor, std::vector<float> & mvInvScaleFactor,
                                std::vector<float> & mvLevelSigma2, std::vector<float> &mvInvLevelSigma2) override;
protected:
    cv::Ptr<cv::SIFT> sift;

    int nfeatures;
    int nOctaveLayers;
    double contrastThreshold;
    double edgeThreshold;
    double sigma;
};

} // ORB_SLAM3

#endif //SIFT_EXTRACTOR_H
