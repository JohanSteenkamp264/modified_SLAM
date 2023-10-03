#include "Extractors/PythonCPPlink.h"

#include <iostream>

using namespace std;

namespace ORB_SLAM3
{    
    cpp_python_link::cpp_python_link(int port){     
        server_port = port;
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        server_address.sin_family = AF_INET;
        server_address.sin_addr.s_addr = INADDR_ANY;
        server_address.sin_port = htons(server_port);  // Choose a port number
        bind(server_socket, (struct sockaddr*)&server_address, sizeof(server_address));
        listen(server_socket, 1);

        /*
        cout << "Starting thread for feature extraction" << endl;

        */
        
        cout << "Waiting for feature description methods to initialize on port " << server_port << endl;
        client_socket = accept(server_socket, nullptr, nullptr);
        cout << "Socket connected to feature extractor" << endl;
    }


    cpp_python_link::~cpp_python_link(){
        close(client_socket);
        close(server_socket);
    }

    bool cpp_python_link::send_image(const cv::Mat &image)
    {
        uint32_t height = image.rows;
        uint32_t width = image.cols;
        uint32_t channels = image.channels();
        
        uint8_t header[12] = {0};
        header[3] = (uint8_t)((height>>24)&0xFF);
        header[2] = (uint8_t)((height>>16)&0xFF);
        header[1] = (uint8_t)((height>>8)&0xFF);
        header[0] = (uint8_t)((height)&0xFF);

        header[7] = (uint8_t)((width>>24)&0xFF);
        header[6] = (uint8_t)((width>>16)&0xFF);
        header[5] = (uint8_t)((width>>8)&0xFF);
        header[4] = (uint8_t)((width)&0xFF);


        header[11] = (uint8_t)((channels>>24)&0xFF);
        header[10] = (uint8_t)((channels>>16)&0xFF);
        header[9] = (uint8_t)((channels>>8)&0xFF);
        header[8] = (uint8_t)((channels)&0xFF);

        // Sending header
        send(client_socket, header, 12, 0);
        uint32_t num_bytes = height*width*channels;
        send(client_socket, image.data, num_bytes, 0);

        return true;
    }

    bool cpp_python_link::recieve_features(std::vector<cv::KeyPoint> &kps,cv::Mat &des)
    {
        uint8_t size_buffer[8] = {0};
        recv(client_socket, size_buffer, 8, MSG_WAITALL);
        uint32_t num_keypoints = size_buffer[0] + (size_buffer[1]<<8) + (size_buffer[2]<<16) + (size_buffer[3]<<24);
        uint32_t descriptor_length = size_buffer[4] + (size_buffer[5]<<8) + (size_buffer[6]<<16) + (size_buffer[7]<<24);

        kps.clear();
        kps.resize(num_keypoints);
        
        des = cv::Mat(num_keypoints, descriptor_length, CV_32F);

        cv::Mat keypoints(num_keypoints, 7, CV_32F);

        recv(client_socket, keypoints.data, num_keypoints*28, MSG_WAITALL);

        uint32_t descriptor_bytes = num_keypoints * descriptor_length * 4;
        recv(client_socket, des.data, descriptor_bytes, MSG_WAITALL);

        float* fl_kp = keypoints.ptr<float>();
        
        for( uint32_t i = 0; i < num_keypoints; i++)
        {
            kps[i].pt.x = *fl_kp;
            fl_kp++;
            kps[i].pt.y = *fl_kp;
            fl_kp++;
            kps[i].size = *fl_kp;
            fl_kp++;
            kps[i].angle = *fl_kp;
            fl_kp++;
            kps[i].response = *fl_kp;
            fl_kp++;
            kps[i].octave = static_cast<int>(*fl_kp);
            fl_kp++;
            kps[i].class_id = static_cast<int>(*fl_kp);
            fl_kp++;
        }
        
        return true;
    }

} // namespace ORB_SLAM3