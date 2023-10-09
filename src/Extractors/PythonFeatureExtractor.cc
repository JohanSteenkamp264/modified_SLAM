#include "Extractors/PythonFeatureExtractor.h"
#include "System.h"
#include <string>

using namespace std;

namespace ORB_SLAM3
{
    cpp_python_link *PythonFeature::link = nullptr;
    std::mutex PythonFeature::mMutexLink;
    thread *PythonFeature::thSuperpoint = nullptr;

    PythonFeature::PythonFeature(int _nfeatures)
    :nfeatures(_nfeatures)
    {
        mMutexLink.lock();
        if(link == nullptr){
        thSuperpoint = new thread(&ORB_SLAM3::PythonFeature::runSuperpoint,this);
           link = new cpp_python_link(12000);
        }
        mMutexLink.unlock();
    }

    bool PythonFeature::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors, cv::Mat &globalDescriptors,
                        int nKeypointsNum, float threshold) 
    {
        mMutexLink.lock();
        cv::Mat tDescriptors;
        link->send_image(image);

        link->recieve_features(vKeyPoints, tDescriptors);
        mMutexLink.unlock();        

        /*if(vKeyPoints.size() > nKeypointsNum)
        {
            tDescriptors = tDescriptors.rowRange(0,nKeypointsNum);
            vKeyPoints.resize(nKeypointsNum);
        }

        for(size_t i = 0; i < vKeyPoints.size(); i++){
            cout << vKeyPoints[i].octave << endl;
        }*/

        tDescriptors.copyTo(localDescriptors);

        globalDescriptors = cv::Mat(4096, 1, CV_32F);
        float* vResGlobalDescriptor = tDescriptors.ptr<float>(0);
        for (int temp = 0; temp < 4096; ++temp)
        {
            globalDescriptors.ptr<float>(0)[temp] = vResGlobalDescriptor[temp];
        }

        return true;
    }

    bool PythonFeature::Detect(const cv::Mat &image, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &localDescriptors,
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

    bool PythonFeature::Detect(const cv::Mat &intermediate, cv::Mat &globalDescriptors)
    {
        intermediate.copyTo(globalDescriptors);
        return false;
    }

    void PythonFeature::runSuperpoint(void){
        string cmd = "bash -c 'python3 ./python_features/PythonFeature.py 12000 " + System::SettingsFile + "'";
        system(cmd.c_str());
    }
} //namespace ORB_SLAM3