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
import os
chessboardSize = (9,6)
frameSize = (1440,1080)
def calibrate():#function to do the camera calibration
    os.chdir('/home/comnets/workspace/hassineThesis/robolab/pycode/calibration_2/')
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
    size_of_chessboard_squares_mm = 24
    objp = objp * size_of_chessboard_squares_mm
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob('*.jpg')
    #print(images)
    for image in images:
        img = cv2.imread(image)
        #print(img.shape)
        img=cv2.resize(img,(1000,700))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img=cv.resize(img,(1000,700))
        #print(img.shape)
        #cv.imshow('img', img)
        #cv.waitKey(0)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)
        print("corners found*************")
        #print(len(corners))
        # If found, add object points, image points (after refining them)
        if ret == True:
            print("yes detected")
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(1000)
            #break
        else :
            print("no detected")    
    cv2.destroyAllWindows()
    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)
    return cameraMatrix, dist, rvecs, tvecs,objpoints, imgpoints

def measure_error(objpoints,imgpoints,rvecs,tvecs, cameraMatrix, dist):# Reprojection Error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )  

if __name__ == "__main__":
    cameraMatrix, dist, rvecs, tvecs,objpoints, imgpoints=calibrate()
    measure_error(objpoints,imgpoints,rvecs,tvecs, cameraMatrix, dist)
    print(cameraMatrix)
    cameraMatrix = np.array([[1.09706045e+04,0.00000000e+00,5.22869579e+02],
    [0.00000000e+00,4.94207851e+03,2.70272151e+02],
    [0.00000000e+00,0.00000000e+00,1.00000000e+00]])