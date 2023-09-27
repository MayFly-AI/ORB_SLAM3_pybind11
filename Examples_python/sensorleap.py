import os
import sys
import time
import numpy as np
import cv2

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../lib') # .so file needs to be in sys.path
import pyorbslam3

from mayfly.videocapture import VideoCapture

def main():
    if len(sys.argv) < 3:
        print("Usage: ./sensorleap.py path_to_vocabulary path_to_settings") 
        exit(0)

    SLAM = pyorbslam3.System(sys.argv[1],sys.argv[2],pyorbslam3.System.eSensor.MONOCULAR,True,0,'')
    imageScale = SLAM.GetImageScale()

    cap = VideoCapture(list(range(64)), '')
    tframe = 1694088977000*1e6
    frame_time_ns = int((1000./30.)*1.e6)
    frame_idx = 0
    while True:
        frames = cap.read()
        frm = frames[0] # It may have more than 1 frame if sync cameras or ToF. We assume 1 frame
        arr = np.from_dlpack(frm['image']).copy()
        bgr = cv2.cvtColor(arr[:,:,:3], cv2.COLOR_RGB2BGR)
        im = cv2.cvtColor(arr[:,:,:3], cv2.COLOR_RGB2GRAY)

        SLAM.TrackMonocular(im, tframe, [], "")
        tframe = tframe+frame_time_ns

        res = SLAM.DrawFrame()
        cv2.imshow('res',res)
        cv2.imshow('bgr',bgr)
        cv2.waitKey(1)
        frame_idx += 1

    output.release()
    SLAM.Shutdown();

if __name__ == "__main__":
    main()
