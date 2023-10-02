#ifndef BASEMODEL_H
#define BASEMODEL_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM3
{

enum ModelType {
    oCVSIFTModel,
    oCVSURFModel,
    oCVKAZEModel
};

enum ModelDetectionMode {
    kImageToLocalAndGlobal,
    kImageToLocal,
    kImageToLocalAndIntermediate,
    kIntermediateToGlobal
};

const std::string gStrModelDetectionName[] = {"ImageToLocalAndGlobal", "ImageToLocal", "ImageToLocalAndIntermediate", "IntermediateToGlobal"};

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class BaseModel
{
public:
    virtual ~BaseModel(void) = default;
    
    virtual bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) = 0;

    virtual bool Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold) = 0;

    virtual bool Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors) = 0;

    virtual bool IsValid(void) = 0;

    virtual ModelType Type(void) = 0;
    
    virtual bool getScaleValues(float &scaleFactor, int &nLevels, std::vector<float> & mvScaleFactor, std::vector<float> & mvInvScaleFactor,
                                std::vector<float> & mvLevelSigma2, std::vector<float> &mvInvLevelSigma2) = 0;
};


class Settings;

void InitAllModels(Settings* settings);

std::vector<BaseModel*> GetModelVec(void);

BaseModel* GetGlobalModel(void);

BaseModel* InitSIFTModel(ModelDetectionMode mode, Settings* settings);

BaseModel* InitSURFModel(ModelDetectionMode mode, Settings* settings);

BaseModel* InitKAZEModel(ModelDetectionMode mode, Settings* settings);

std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int minX,
                                           const int maxX, const int minY, const int maxY, const int N);

std::vector<cv::KeyPoint> NMS(const std::vector<cv::KeyPoint> &vToDistributeKeys, int width, int height, int radius);

void Resampler(const float* data, const float* warp, float* output,
                const int batch_size, const int data_height, 
                const int data_width, const int data_channels, const int num_sampling_points);

} // namespace ORB_SLAM3

#endif
