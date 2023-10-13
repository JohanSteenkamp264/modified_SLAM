from pathlib import Path
import numpy as np
import cv2
import tensorflow as tf
from threading import RLock

from models.matching import Matching
from utils.utils import load_img, get_img_paths, check_args, make_matching_plot_fast, AverageTimer, pre_process
from models.zippypoint import load_ZippyPoint
from models.postprocessing import PostProcessing


def is_opencv_version_greater_equal(a, b, c):
    opencv_version = get_opencv_version()
    return opencv_version[0]*1000 + opencv_version[1]*100 + opencv_version[2] >= a*1000 + b*100 + c

class ZippyPointFeature2D:
    def __init__(self,resize = [640,480], max_keypoints = -1,keypoint_threshold = 0.0001, nms_window = 3, ratio_threshold = 0.95, keypoints_size = 20, do_cuda=True  ) -> None:

        self.resize = resize
        self.max_keypoints = max_keypoints
        self.keypoint_threshold = keypoint_threshold
        self.nms_window = nms_window
        self. ratio_threshold = ratio_threshold
        self.do_cuda = do_cuda
        self.keypoints_size = keypoints_size

        self.lock = RLock()
        config_superpoint = {
            'nms_radius': self.nms_window,
            'keypoint_threshold': self.keypoint_threshold,
            'max_keypoints': self.max_keypoints
        }

        pretrained_path = './models/weights'
        self.ZippyPoint = load_ZippyPoint(pretrained_path, input_shape = self.resize)
        self.post_processing = PostProcessing(nms_window=self.nms_window,
                                     max_keypoints=self.max_keypoints,
                                     keypoint_threshold=self.keypoint_threshold)
        self.config_matching = {
            'do_mutual_check': True,
            'ratio_threshold': self.ratio_threshold,
        }

        self.keys = ['keypoints', 'scores', 'descriptors']

    def process_resize(self, w, h, resize):
        assert(len(resize) > 0 and len(resize) <= 2)
        if len(resize) == 1 and resize[0] > -1:
            scale = resize[0] / max(h, w)
            w_new, h_new = int(round(w*scale)), int(round(h*scale))
        elif len(resize) == 1 and resize[0] == -1:
            w_new, h_new = w, h
        else:  # len(resize) == 2:
            w_new, h_new = resize[0], resize[1]

        # Issue warning if resolution is too small or too large.
        if max(w_new, h_new) < 160:
            print('Warning: input resolution is very small, results may vary')
        elif max(w_new, h_new) > 2000:
            print('Warning: input resolution is very large, results may vary')

        return w_new, h_new

    def detectAndCompute(self, frame, mask=None):
        with self.lock: 
            if frame is None:
                raise Exception('Error reading image')
            

            w, h = frame.shape[1], frame.shape[0]

            w_new, h_new = self.process_resize(w, h, self.resize)
            frame = cv2.resize(frame, (w_new, h_new), interpolation=cv2.INTER_AREA)

            print(frame.shape)

            frame_tensor, img_pad = pre_process(frame)
            scores, keypoints, descriptors = self.ZippyPoint(frame_tensor, False)
            scores, keypoints, descriptors = self.post_processing(scores, keypoints, descriptors)
            # Correct keypoint location given required padding
            keypoints -= tf.constant([img_pad[2][0], img_pad[1][0]], dtype=tf.float32)

            scores = scores[0].cpu().numpy()
            keypoints = keypoints[0].cpu().numpy()
            descriptors = np.array(descriptors[0].cpu().numpy(), dtype=np.float32)

            kps = [ cv2.KeyPoint(keypoints[i][0], keypoints[i][1], _size=self.keypoints_size, _response=scores[i]) for i in range(len(scores))]  

            return kps, descriptors

            



