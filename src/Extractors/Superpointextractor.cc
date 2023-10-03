#include "Extractors/Superpointextractor.h"

using namespace std;

namespace ORB_SLAM3
{
    cpp_python_link *SuperPointModel::link = nullptr;
    std::mutex SuperPointModel::mMutexLink;
    thread *SuperPointModel::thSuperpoint = nullptr;

    SuperPointModel::SuperPointModel(int _nfeatures)
    :nfeatures(_nfeatures)
    {
        mMutexLink.lock();
        if(link == nullptr){
        thSuperpoint = new thread(&ORB_SLAM3::SuperPointModel::runSuperpoint,this);
           link = new cpp_python_link(12000);
        }
        mMutexLink.unlock();
    }

    bool SuperPointModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) 
    {
        mMutexLink.lock();
        cv::Mat tDescriptors;
        link->send_image(image);

        link->recieve_features(vKeyPoints, tDescriptors);
        mMutexLink.unlock();        

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

    bool SuperPointModel::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
                        int nKeypointsNum, float threshold)
    {
        mMutexLink.lock();
        cv::Mat tDescriptors;
        link->send_image(image);

        link->recieve_features(vKeyPoints, tDescriptors);
        mMutexLink.unlock();

        if(vKeyPoints.size() > nKeypointsNum)
        {
            tDescriptors = tDescriptors.rowRange(0,nKeypointsNum);
            vKeyPoints.resize(nKeypointsNum);
        }

        tDescriptors.copyTo(localDescriptors);
        return true;
    }

    bool SuperPointModel::Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors)
    {
        intermediate.copyTo(globalDescriptors);
        return false;
    }

    void SuperPointModel::runSuperpoint(void){
        system("bash -c 'python3 ./python_features/feature_Superpoint.py 12000'");
    }
} //namespace ORB_SLAM3