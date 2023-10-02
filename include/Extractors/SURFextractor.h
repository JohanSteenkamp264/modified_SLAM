#ifndef SURF_EXTRACTOR_H
#define SURF_EXTRACTOR_H

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Extractors/BaseModel.h"
namespace ORB_SLAM3
{

class SURFModel: public BaseModel
{
public:
    SURFModel(Settings* settings){};
    ~SURFModel(){};
    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors) override;

    bool IsValid(void) override { return true; };

    ModelType Type(void) override {return oCVSURFModel;};
    
    bool getScaleValues(float &scaleFactor, std::vector<float> & mvScaleFactor, std::vector<float> & mvInvScaleFactor, std::vector<float> & mvLevelSigma2, std::vector<float> &mvInvLevelSigma2) {return true;};
};

} // ORB_SLAM3

#endif //SURF_EXTRACTOR_H
