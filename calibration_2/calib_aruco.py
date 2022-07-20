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

from frankx import Robot, RobotMode, Affine, MotionData, Reaction, Measure, JointMotion, ImpedanceMotion, StopMotion, PathMotion, LinearMotion, WaypointMotion, Waypoint, LinearRelativeMotion
chessboardSize = (9,6)
frameSize = (1440,1080)
'''
def calibrate():#function to do the camera calibration
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
        error = cv2.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )  '''  
def robot_setup(dyn_rel = 0.25):
    robot.set_default_behavior()
    robot.set_EE((0.7071, -0.7071, 0.0, 0.0, -0.7071, -0.7071, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.1034, 1.0))##Gripper-Default-EE
    robot.set_load(0.73, (-0.01, 0.0, 0.03), (0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017)) #Gripper-Default-Load
    robot.set_dynamic_rel(dyn_rel)
# cameraMatrix   
cameraMatrix = np.array([[1.09706045e+04,0.00000000e+00,5.22869579e+02],
    [0.00000000e+00,4.94207851e+03,2.70272151e+02],
    [0.00000000e+00,0.00000000e+00,1.00000000e+00]])
'''default_cali_points = [[0.0,0.05,-0.1,0.0],[0.0,-0.05,-0.1,0.0],
                       [0.1,0.1,-0.05,0.0],[0.2,-0.1,-0.1,0.0],
                       [-0.2,0.05,-0.05,0.0],[0.1,-0.05,-0.07,0.0],
                       [-0.1,0.025,-0.07,0.0],[0.2,-0.025,-0.07,0.0]]  
default_cali_points = [[-0.2,0.13,-0.12,0.0],[0.2,0.15,-0.12,0.0],
                       [0.0,0.1,-0.12,0.0],[0.0,-0.05,-0.1,0.0],
                       [0.15,0.05,-0.12,0.0],[-0.1,-0.02,-0.12,0.0],
                       [0.1,0.14,-0.12,0.0],[-0.15,0.0,-0.12,0.0]]   '''
default_cali_points = [[0.0,0.18,-0.12,0.0],[0.0,0.1,-0.1,0.0],
                        [0.13,0.1,-0.12,0.0],[-0.13,0.1,-0.12,0.0],
                        [0.13,0.0,-0.12,0.0],[-0.13,0.0,-0.12,0.0],
                       [0.1,0.1,-0.12,0.0],[-0.1,0.18,-0.12,0.0]]                      
np_cali_points = np.array(default_cali_points)
arm_cord = np.column_stack((np_cali_points[:,0:3], np.ones(np_cali_points.shape[0]).T)).T
def calculate_XYZ(u,v,camera_matrix):
    uv_1=np.array([[u,v,1]], dtype=np.float32)
    uv_1=uv_1.T
    cam_mtx =  camera_matrix
    inverse_cam_mtx = np.linalg.inv(cam_mtx)
    XYZ=inverse_cam_mtx.dot(uv_1)
    x,y,z=XYZ[0,0],XYZ[1,0],XYZ[2,0]
    return x,y,z
def recover_error(motion_forward=''):
    if robot.read_once().robot_mode == RobotMode.Reflex :
        print("recover_from_errors")
        robot.recover_from_errors()
        time.sleep(0.5)
        if  motion_forward != '':
            robot.move(motion_forward)
            print('moving....')
            if robot.read_once().robot_mode == RobotMode.Reflex :
                recover_error(motion_forward)
                time.sleep(0.5)
        time.sleep(0.5)
if __name__ == "__main__":
    #cameraMatrix, dist, rvecs, tvecs,objpoints, imgpoints=calibrate()
    #measure_error(objpoints,imgpoints,rvecs,tvecs, cameraMatrix, dist)
    print(cameraMatrix)
    parser = ArgumentParser()
    parser.add_argument("--robotip", default="192.168.1.2", help="robot ip")
    parser.add_argument("--gatewayip", default="192.168.2.1", help="gateway ip")
    args = parser.parse_args()


    # Connect to the robot
    global robot, gripper, is_master_mode
    robot = Robot(fci_ip = args.robotip, repeat_on_error = False)
    robot.set_default_behavior()
    robot.recover_from_errors()

    # Reduce the acceleration and velocity dynamic
    robot.velocity_rel = 0.2
    robot.acceleration_rel = 0.02
    robot.jerk_rel = 0.001
    gripper = robot.get_gripper()
    gripper.gripper_force = 50.0
    
    #joint_motion = JointMotion([-2.4417100295260634, 1.0681055992583837, 2.2147164654894858, -2.893747881959243, -0.8530632804061193, 1.936590144773569, 1.1926808407695957])
    joint_motion = JointMotion([-0.0046692655346566615, -0.43185659684792743, -0.00342779850873683, -2.736275297599926, 0.023197775555373598, 2.3191364502928216, 0.762515997321423])
    robot.move(joint_motion)
    gripper.open()
    print(cameraMatrix)
    np_cali_points = np.array(default_cali_points)
    arm_cord = np.column_stack((np_cali_points[:,0:3], np.ones(np_cali_points.shape[0]).T)).T    
    centers=np.ones(arm_cord.shape)
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) 
    for ind,pt in enumerate(default_cali_points):
        print("Current points:", pt)
        # Define and move forwards
        way = Affine(*pt)
        motion_forward = LinearRelativeMotion(way)
        robot.move(motion_forward)
        recover_error(motion_forward)
        '''gripper.clamp()
        way = Affine(0.0,0.0,-0.15 )
        motion_forward = LinearRelativeMotion(way)
        robot.move(motion_forward)
        recover_error(motion_forward)
        gripper.open()
        motion_backward = LinearRelativeMotion(way.inverse())
        robot.move(motion_backward) 
        recover_error(motion_forward)
        time.sleep(3)'''
        while True:
            _, video_frame = cap.read() 
            gray = cv2.cvtColor(video_frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            frame_markers = aruco.drawDetectedMarkers(video_frame.copy(), corners, ids)
            aruco.drawDetectedMarkers(video_frame, corners,ids) #Draw a square around the mark
            try:
                if len(ids)!= 0:
                    index = list(ids).index([3])
                c = corners[index][0]
                a,b=int(c[:, 0].mean()), int(c[:, 1].mean()) 
                x,y,z= calculate_XYZ(a,b,cameraMatrix)
                print("Current points:", pt)
                print(a,b)
                print(x,y,z)
                cv2.circle(video_frame, (a,b), 4, (0, 0, 255), -1)   
            except Exception:
                print('error')
            cv2.imshow('video_frame',video_frame)
            if cv2.waitKey(1) == ord('a'):
                '''motion_backward = LinearRelativeMotion(way.inverse())
                robot.move(motion_backward)'''
                cv2.destroyAllWindows()
                joint_motion = JointMotion([-0.0046692655346566615, -0.43185659684792743, -0.00342779850873683, -2.736275297599926, 0.023197775555373598, 2.3191364502928216, 0.762515997321423])
                robot.move(joint_motion)
                recover_error()
                #time.sleep(3)
                centers[0:3,ind]=[x,y,z]
                break
                    
    image_to_arm = np.dot(arm_cord, np.linalg.pinv(centers))    
    print("Finished")
    print("Image to arm transform:\n", image_to_arm)
    print("Sanity Test:")
    print("-------------------")
    print("Image_to_Arm")
    print("-------------------")
    for ind, pt in enumerate(centers.T):
        print("Expected:", default_cali_points[ind][0:3])
        #print(ind,pt)
        print("Result:", np.dot(image_to_arm, np.array(pt))[0:3])
        #print(c)         
        '''a,b=int(c[:, 0].mean()), int(c[:, 1].mean())
        x,y,z= calculate_XYZ(a,b,cameraMatrix)
        cv2.circle(video_frame, (a,b), 4, (0, 0, 255), -1)
        cv2.imshow('video_frame',video_frame)
        cv2.waitKey(1000)
        print(a,b)
        print(x,y,z)
        motion_backward = LinearRelativeMotion(way.inverse())
        robot.move(motion_backward)
        cv2.destroyAllWindows()'''

