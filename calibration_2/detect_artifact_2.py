import numpy as np
import cv2
from cv2 import aruco
import time
import glob
from argparse import ArgumentParser
from time import sleep
import math
import socket
import threading
import signal
from argparse import ArgumentParser
from detect_artifact import RecogniseArtifact
image_to_arm=np.array([[-3.26119036e+00 , 1.68067199e+00 , 1.27566016e-02  ,1.27566016e-02],
 [ 2.63740785e+00 , 1.97309011e+00 ,-2.29012727e-03 ,-2.29012727e-03],
 [-1.64744823e-01,  1.60628065e-01 ,-4.87132832e-02 ,-4.87132832e-02],
 [-7.99360578e-15 ,-1.06581410e-14 , 5.00000000e-01 , 5.00000000e-01]])
# cameraMatrix   
cameraMatrix = np.array([[1.09706045e+04,0.00000000e+00,5.22869579e+02],
    [0.00000000e+00,4.94207851e+03,2.70272151e+02],
    [0.00000000e+00,0.00000000e+00,1.00000000e+00]])
artifact_jid ="move_artifact2@comnets-pc08"
artifact_passwd = "admin"
artifact2 = RecogniseArtifact(artifact_jid, artifact_passwd,id_camera=2,artifact_window='video_frame_2',cameraMatrix=cameraMatrix,image_to_arm=image_to_arm)
future = artifact2.start()
future.result()
artifact2.join()