#ifndef SURF_EXTRACTOR_H
#define SURF_EXTRACTOR_H

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "Extractors/BaseModel.h"
namespace ORB_SLAM3
{

class SURFModel: public BaseModel
{
public:
    SURFModel(double _hessianThreshold, int _nOctaves, int _nOctaveLayers, bool _extended, bool _upright);
    ~SURFModel(){};
    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors) override;

    bool IsValid(void) override { return true; };

    ModelType Type(void) override {return oCVSURFModel;};
private:
    double hessianThreshold;
    int nOctaves;
    int nOctaveLayers;
    bool extended;
    bool upright;

    cv::Ptr<cv::xfeatures2d::SURF> surf;
};

} // ORB_SLAM3

#endif //SURF_EXTRACTOR_H
