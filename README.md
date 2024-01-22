# Modified SLAM
This is a modified version of [HFNet-SLAM](https://github.com/LiuLimingCode/HFNet_SLAM) to use other feature methods including SIFT, SURF and KAZE. Future includes Deep Learning methods from [PySLAM](https://github.com/luigifreda/pyslam) by employing a cpp_python communication. Loop closure detection is disabled due to the testing done on simulated subteranean datasets which caused false loop closures on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). The global descriptor is replaced by the concatanation of the highest scoring detected features untuil the length of the descriptor is reached.

# Instalation Instructions

```
cd 'your-workspace'
```

```
git clone --recursive https://github.com/JohanSteenkamp264/modified_SLAM.git
```


## HF-Net SLAM instalation
We use OpenCV to manipulate images and features.

```
sudo apt-get install libopencv-dev
```

## Building Modified-SLAM library and examples

```
chmod +x build.sh
bash build.sh
```

## Pyslam
```
cd Thirdparty
git clone --recursive https://github.com/luigifreda/pyslam.git
cd pyslam
./install_all.sh

```

# 2. Prerequisites
We have tested the library in **Ubuntu 18.04**, but it should be easy to compile in other platforms. 

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.
**SURF**
  in order to use SURF features, which is patented [OpenCV](http://opencv.org) needs to be build from source with non-free modules enabled, see the following [guide](https://drthitirat.wordpress.com/2019/01/20/opencv-python-build-opencv-4-0-1-dev-contrib-non-free-siftsurf-from-sources-on-windows-10-64-bit-os/).

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Python
Required to for Python features, version used in testing is **3.6**. [Tensorflow](https://www.tensorflow.org/install) and [Pytorch](https://pytorch.org/get-started/locally/) is used in deep leanned features and descriptor implimetations, mainly from [PySLAM](https://github.com/luigifreda/pyslam), with [Zippypoint](https://github.com/menelaoskanakis/ZippyPoint) implemeted seperate. 

## [PySLAM](https://github.com/luigifreda/pyslam)
We use the feature extraction and descriptors of Pyslam including deep learned features such as R2D2, D2Net, Superpoint and DELF. For all required prerequisites please follow the [PySLAM](https://github.com/luigifreda/pyslam) guides.

# Adding Features
In the ***PythoonFeature.py*** file
```Python
    if settings["Extractor.Python.type"].upper() == "SUPERPOINT":
        feature = inti_superpoint(settings)
    elif settings["Extractor.Python.type"].upper() == "DELF":
        feature = inti_delf(settings)
    elif settings["Extractor.Python.type"].upper() == "R2D2":
        feature = init_r2d2(settings)
    elif settings["Extractor.Python.type"].upper() == "D2NET":
        feature = init_d2net(settings)
    elif settings["Extractor.Python.type"].upper() == "LFNET":
        feature = init_lfnet(settings)
    elif settings["Extractor.Python.type"].upper() == "ZIPPYPOINT":
        feature = init_zippypoint(settings)
    else:
        print(f"Invalid Extractor.Python.type {settings['Extractor.Python.type']}")
        exit()
```
add an additional elif statement with yout feature name
```Python
    if settings["Extractor.Python.type"].upper() == "SUPERPOINT":
        feature = inti_superpoint(settings)
    elif settings["Extractor.Python.type"].upper() == "DELF":
        feature = inti_delf(settings)
    elif settings["Extractor.Python.type"].upper() == "R2D2":
        feature = init_r2d2(settings)
    elif settings["Extractor.Python.type"].upper() == "D2NET":
        feature = init_d2net(settings)
    elif settings["Extractor.Python.type"].upper() == "LFNET":
        feature = init_lfnet(settings)
    elif settings["Extractor.Python.type"].upper() == "ZIPPYPOINT":
        feature = init_zippypoint(settings)
    elif settings["Extractor.Python.type"].upper() == "MYFEATURE":
        feature = init_myfeature(settings)
    else:
        print(f"Invalid Extractor.Python.type {settings['Extractor.Python.type']}")
        exit()
```
note this also requires a ***init_myfeature*** fucntion
```Python
def init_myfeature(settings):
    from feature_myfeature import MyFeature2D
    return MyFeature2D()
```
Here **MyFeature2D** acts similat to the OpenCV's Feature2D with the ***detectandcompute*** function returning a list of OpenCV keypoints, and a 2D aray of descriptors, as the following exaple of using OpenCV's ORB as feature.
```Python
class MyFeature2D
  def __init__(self):
    self.orb = cv.ORB_create()

  def detectandcompute(self,frame,mask=None):
    kps,des = self.orb.detectandcompute(frame)
  return kps, des
 
```

