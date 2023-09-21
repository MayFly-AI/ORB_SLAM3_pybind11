import os
import sys
import time
import numpy as np
import cv2

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../lib') # .so file needs to be in sys.path
import pyorbslam3

def LoadImages(strImagePath, strPathTimes):
    strImages = []
    TimeStamps = []
    fTimes = open(strPathTimes, 'r')
    for s in fTimes.readlines():
        if len(s) != 0:
            ss = int(s)
            strImages.append(strImagePath + "/" + str(ss) + ".png")
            TimeStamps.append(float(ss)*1e-9)
    return strImages, TimeStamps

def main():
    if len(sys.argv) < 5:
        print("Usage: ./mono_euroc.py path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)") 
        exit(0)

    num_seq = (len(sys.argv)-3)//2
    print("num_seq = ", num_seq)
    bFileName = (len(sys.argv)-3 % 2) == 1
    file_name = None
    if bFileName: # optional trajectory_file_name
        file_name = sys.argv[len(sys.argv)-1]
        print("file name: ", file_name)
    
    seq = 0
    vstrImageFilenames = []
    vTimestampsCam = []
    nImages = []
    tot_images = 0
    for seq in range(0,num_seq):
        print("Loading images for sequence ", seq, "...")
        strImageFilenames, TimestampsCam = LoadImages(sys.argv[2*seq+3]+"/mav0/cam0/data", sys.argv[2*seq+4])
        vstrImageFilenames.append(strImageFilenames)
        vTimestampsCam.append(TimestampsCam)
        print('LOADED!')
        nImages.append(len(strImageFilenames))
        tot_images += len(strImageFilenames)

    vTimesTrack = []
    print("---------")
    fps = 20
    dT = 1./fps

    SLAM = pyorbslam3.System(sys.argv[1],sys.argv[2],pyorbslam3.System.eSensor.MONOCULAR,True,0,'')
    imageScale = SLAM.GetImageScale()

    t_resize = 0.;
    t_track = 0.;
    for seq in range(0,num_seq):
        proccIm = 0
        for ni in range(0,nImages[seq]):
            im = cv2.imread(vstrImageFilenames[seq][ni], cv2.IMREAD_UNCHANGED)
            tframe = vTimestampsCam[seq][ni]
            proccIm += 1
            if imageScale != 1.:
                print('resizing')
                width = im.shape[1] * imageScale;
                height = im.shape[0] * imageScale;
                cv2.resize(im, im, cv2.Size(width, height));

            t1 = time.time()
            SLAM.TrackMonocular(im, tframe, [], "")
            t2 = time.time()

            ttrack = t2-t1

            vTimesTrack.append(ttrack)

            # Wait to load the next frame
            T=0
            if ni<nImages[seq]-1:
                T = vTimestampsCam[seq][ni+1]-tframe
            elif ni>0:
                T = tframe-vTimestampsCam[seq][ni-1]

            if ttrack<T: 
                time.sleep(T-ttrack)

        if seq < num_seq - 1:
            kf_file_submap =  "./SubMaps/kf_SubMap_" + str(seq) + ".txt";
            f_file_submap =  "./SubMaps/f_SubMap_" + str(seq) + ".txt";
            SLAM.SaveTrajectoryEuRoC(f_file_submap);
            SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);
            print("Changing the dataset")
            SLAM.ChangeDataset();

    SLAM.Shutdown();

    # Save camera trajectory
    if bFileName:
        kf_file =  "kf_" + sys.argv[argc-1] + ".txt";
        f_file =  "f_" + sys.argv[argc-1] + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    else:
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

if __name__ == "__main__":
    main()
