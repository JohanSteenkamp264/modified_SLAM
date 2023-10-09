import socket
import numpy as np
import cv2
import time
import os
import sys
import yaml

print(os.getcwd())

def read_parameters(settings_file):
    with open(settings_file, "r") as file:
        yaml_content = file.read().splitlines()[1:]
    yaml_content = "\n".join(yaml_content)
    data = yaml.safe_load(yaml_content)

    return data

def inti_superpoint(settings):
    from feature_superpoint import SuperPointFeature2D
    os.chdir("./Thirdparty/pyslam/")
    return SuperPointFeature2D()

def inti_delf(settings):
    from feature_delf import DelfFeature2D
    os.chdir("./Thirdparty/pyslam/")
    return DelfFeature2D()

def init_r2d2(settings):
    from feature_r2d2 import R2d2Feature2D
    os.chdir("./Thirdparty/pyslam/")
    return R2d2Feature2D()

def init_d2net(settings):
    from feature_d2net import D2NetFeature2D
    os.chdir("./Thirdparty/pyslam/")
    return D2NetFeature2D()

def init_lfnet(settings):
    from feature_lfnet import LfNetFeature2D
    os.chdir("./Thirdparty/pyslam/")
    return LfNetFeature2D()


def init_keynet(settings):
    from feature_root_sift import RootSIFTFeature2D
    os.chdir("./Thirdparty/pyslam/")
    return RootSIFTFeature2D(cv2.SIFTcreate())


class socket_feature:
    def __init__(self, port, feature_extractor) -> None:
        self.feature_extractor = feature_extractor

        print("Connecting to port ", port)	
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('localhost', int(port))
        
        connected = False
        while not connected:
            try:
                self.client_socket.connect(self.server_address)
                connected = True
            except:
                time.sleep(0.1)
                pass

    def run(self) -> None:
        shape = np.zeros(3, dtype=np.uint32)
        num_bytes = 0.0
        while True:
            header = self.client_socket.recv(12)
            if len(header) == 0:
                print("Server socket closed")
                self.client_socket.close()
                return;

            shape = np.frombuffer(header, dtype = np.uint32)
            num_bytes = shape[0]*shape[1]*shape[2]

            image = np.zeros((num_bytes),dtype=np.uint8)
            index = 0
            while index < num_bytes:
                rec = np.frombuffer(self.client_socket.recv(shape[0]*shape[2]), dtype=np.uint8)
                image[index:index + len(rec)] = rec
                index += len(rec)
            
            if shape[2] == 1:
                image = image.reshape((shape[0], shape[1])).astype(np.uint8)
            else:
                image = image.reshape((shape[0], shape[1], shape[2])).astype(np.uint8)

            kps, des = self.feature_extractor.detectAndCompute(image, None)
            kp_arr = np.zeros(len(kps)*7, dtype=np.float32)
            for i,kp in enumerate(kps):
                ind = i*7
                kp_arr[ind] = kp.pt[0]
                kp_arr[ind + 1] = kp.pt[1]
                kp_arr[ind + 2] = kp.size
                kp_arr[ind + 3] = kp.angle
                kp_arr[ind + 4] = kp.response
                kp_arr[ind + 5] = kp.octave
                kp_arr[ind + 6] = kp.class_id

            size_bytes = np.array([des.shape[0], des.shape[1]], dtype=np.uint32).tobytes()
            self.client_socket.sendall(size_bytes)

            kp_bytes = kp_arr.tobytes()
            self.client_socket.sendall(kp_bytes)

            self.client_socket.sendall(des.tobytes())

if __name__ == '__main__':
    global kVerbose
    kVerbose = False
    
    settings_file = sys.argv[2]
    # customizze settings
    settings = read_parameters(settings_file)
    sys.path.append("./Thirdparty/pyslam/")
    feature = None
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
    elif settings["Extractor.Python.type"].upper() == "KEYNET":
        feature = init_keynet(settings)
    else:
        print(f"Invalid Extractor.Python.type {settings['Extractor.Python.type']}")
        exit()

    feature_extractor = socket_feature(sys.argv[1], feature)

    print(settings_file)
    feature_extractor.run()
