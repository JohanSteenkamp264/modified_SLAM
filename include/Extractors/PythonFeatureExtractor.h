#ifndef SUPERPOINT_EXTRACTOR_H
#define SUPERPOINT_EXTRACTOR_H

#include <cmath>
#include <vector>
#include <mutex>
#include <thread>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Extractors/BaseModel.h"
#include "Extractors/PythonCPPlink.h"

using namespace std;

namespace ORB_SLAM3
{

class PythonFeature: public BaseModel
{
public:
    PythonFeature(int nfeatures);
    ~PythonFeature(){};
    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold) override;

    bool Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors) override;

    bool IsValid(void) override { return true; };

    ModelType Type(void) override {return PythonFeatureModel;};

protected:
    void runSuperpoint(void);

    static thread *thSuperpoint;
    static cpp_python_link *link;
    static std::mutex mMutexLink;
    int nfeatures;
};

} // ORB_SLAM3

#endif //SUPERPOINT_EXTRACTOR_H
