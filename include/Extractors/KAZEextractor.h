#ifndef KAZE_EXTRACTOR_H
#define KAZE_EXTRACTOR_H

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "Extractors/BaseModel.h"
namespace ORB_SLAM3
{

class KAZEModel: public BaseModel
{
public:
    KAZEModel(bool extended, bool upright, float threshold, int nOctaves, int nOctaveLayers);
    ~KAZEModel(){};
    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors) override;

    bool IsValid(void) override { return true; };

    ModelType Type(void) override {return oCVKAZEModel;};
private:
    bool extended;
    bool upright;
    float threshold;
    int nOctaves;
    int nOctaveLayers;

    cv::Ptr<cv::Feature2D> KAZE;
};

} // ORB_SLAM3

#endif //KAZE_EXTRACTOR_H
