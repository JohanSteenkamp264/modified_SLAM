# Modified SLAM
This repository presents a modified version of [HFNet-SLAM](https://github.com/LiuLimingCode/HFNet_SLAM), adapted to incorporate alternative feature detection methods such as SIFT, SURF, and KAZE. Future updates will include Deep Learning techniques from [PySLAM](https://github.com/luigifreda/pyslam) through cpp_python communication integration. Loop closure detection has been disabled due to tests conducted on simulated subterranean datasets, where it caused false loop closures in ORB- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). Instead of using a global descriptor, this version concatenates the highest scoring detected features until the desired length of the descriptor is achieved.

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
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Tested with OpenCV 4.8.0**.
**SURF**
  To utilize SURF features, which are patented, you must build [OpenCV](http://opencv.org) from source with the non-free modules enabled. Detailed instructions for this process can be found in the following  [guide](https://developer.ridgerun.com/wiki/index.php/Compiling_OpenCV_from_Source). When using the cmake command during this process, be sure to include the following line to enable the non-free modules
  ```bash
  -D OPENCV_ENABLE_NONFREE:BOOL=ON
  ```

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Python
For Python functionality, the version used in testing is **3.6**. We employ  [Tensorflow](https://www.tensorflow.org/install) and [Pytorch](https://pytorch.org/get-started/locally/) for deep learning features and descriptor implementations, primarily sourced from [PySLAM](https://github.com/luigifreda/pyslam). Additionally, [Zippypoint](https://github.com/menelaoskanakis/ZippyPoint) is implemented separately to complement these frameworks.

## [PySLAM](https://github.com/luigifreda/pyslam)
We utilize the feature extraction and descriptor capabilities of [PySLAM](https://github.com/luigifreda/pyslam), including deep learning-based features like R2D2, D2Net, Superpoint, and DELF. To ensure all required prerequisites are met, please follow the guidelines and installation instructions provided in the PySLAM repository.

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
Here **MyFeature2D** acts simmilar to the OpenCV's Feature2D with the ***detectandcompute*** function returning a list of OpenCV keypoints, and a 2D aray of descriptors, as the following exaple of using OpenCV's ORB as feature.
```Python
class MyFeature2D:
  def __init__(self):
    self.orb = cv.ORB_create()

  def detectandcompute(self,frame,mask=None):
    kps,des = self.orb.detectandcompute(frame)
    return kps, des
 
```
To use the new feature, the calibration ***.yaml*** should be modified as follows.
```YAML
Extractor.type: "PYTHON" 
Extractor.Python.type: "MYFEATURE"
```
Also remember to change the thresholds in descriptor distances for accurate matching, in accordance to the added feature.
```YAML
Matcher.TH_HIGH: 1.3
Matcher.TH_LOW: 1.0
```

