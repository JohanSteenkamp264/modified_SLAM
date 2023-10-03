import socket
import numpy as np
import cv2
import time
import os
import sys

print(os.getcwd())

sys.path.append("./Thirdparty/pyslam")

from feature_superpoint import SuperPointFeature2D


class socket_superpoint:
    def __init__(self, port:int) -> None:
        self.feature_extractor = SuperPointFeature2D()

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
        shape = np.zeros(2, dtype=np.float32)
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
    feature_extractor = socket_superpoint(sys.argv[1])
    feature_extractor.run()
