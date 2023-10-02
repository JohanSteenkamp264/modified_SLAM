#ifndef KAZE_EXTRACTOR_H
#define KAZE_EXTRACTOR_H

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Extractors/BaseModel.h"
namespace ORB_SLAM3
{

class KAZEModel: public BaseModel
{
public:
    KAZEModel(Settings* settings){};
    ~KAZEModel(){};
    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors) override;

    bool IsValid(void) override { return true; };

    ModelType Type(void) override {return oCVKAZEModel;};

    bool getScaleValues(float &scaleFactor, std::vector<float> & mvScaleFactor, std::vector<float> & mvInvScaleFactor, std::vector<float> & mvLevelSigma2, std::vector<float> &mvInvLevelSigma2) {return true;};
};

} // ORB_SLAM3

#endif //KAZE_EXTRACTOR_H
