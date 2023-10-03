#ifndef CPP_PYTHON_LINK_H
#define CPP_PYTHON_LINK_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <stdio.h>
#include <stdint.h>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <vector>

namespace ORB_SLAM3
{

class cpp_python_link
{
public:
    cpp_python_link(int port);
    ~cpp_python_link();
    bool send_image(const cv::Mat &image);
    bool recieve_features(std::vector<cv::KeyPoint> &kps, cv::Mat &des);
protected:
    int client_socket;
    int server_port;
    int server_socket;
    struct sockaddr_in server_address;
};
    
} //namespace ORB_SLAM3

#endif