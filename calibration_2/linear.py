from argparse import ArgumentParser

from frankx import Robot, RobotMode, Affine, MotionData, Reaction, Measure, JointMotion, ImpedanceMotion, StopMotion, PathMotion, LinearMotion, WaypointMotion, Waypoint, LinearRelativeMotion
import cv2 as cv
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

def robot_setup(dyn_rel = 0.25):
    robot.set_default_behavior()
    robot.set_EE((0.7071, -0.7071, 0.0, 0.0, -0.7071, -0.7071, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.1034, 1.0))##Gripper-Default-EE
    robot.set_load(0.73, (-0.01, 0.0, 0.03), (0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017)) #Gripper-Default-Load
    robot.set_dynamic_rel(dyn_rel)
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
if __name__ == '__main__':
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
    robot.set_dynamic_rel(0.15)
    # Reduce the acceleration and velocity dynamic
    robot.velocity_rel = 0.2
    robot.acceleration_rel = 0.02
    robot.jerk_rel = 0.001
    gripper = robot.get_gripper()
    gripper.gripper_force = 50.0
    
    #joint_motion = JointMotion([-2.4417100295260634, 1.0681055992583837, 2.2147164654894858, -2.893747881959243, -0.8530632804061193, 1.936590144773569, 1.1926808407695957])
    joint_motion = JointMotion([-0.0046692655346566615, -0.43185659684792743, -0.00342779850873683, -2.736275297599926, 0.023197775555373598, 2.3191364502928216, 0.762515997321423])
    
    robot.move(joint_motion)
    recover_error(joint_motion)
    default_cali_points = [[0.0,0.1,-0.1,0.0],[0.0,0.2,-0.1,0.0],
                       [0.1,0.0,0.0,0.0],[0.2,0.0,0.05,0.0],
                       [0.3,0.1,0.0,0.0],[0.1,0.3,0.1,0.0],
                       [0.1,0.2,0.0,0.0],[0.2,0.1,-0.1,0.0]]
    # To use a given frame relative to the end effector
    way = Affine(-0.13, 0.05,-0.12 )
    m3 = LinearRelativeMotion(way)
    robot.move( m3)   
    recover_error(m3)                
    # Define and move forwards
    #way = Affine(0.1, 0.0, 0.0,0.0)
    #motion_forward = LinearRelativeMotion(way)
    #robot.move(motion_forward)
    #recover_error(motion_forward)
    #m5 = WaypointMotion([
    #Waypoint(Affine(0.1, 0.1, 0.0, 0.0, 0.0, 0.0))
  # The following waypoint is relative to the prior one)
    #])
    #robot.move(m5)
    #way = Affine(0.0,0.0,-0.14 )
    #motion_forward = LinearRelativeMotion(way)
    #robot.move(motion_forward)
    #recover_error(motion_forward)
    '''
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    while True: 
        ret, video_frame = cap.read()
        print(ret)
        gray = cv2.cvtColor(video_frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(video_frame.copy(), corners, ids)
        aruco.drawDetectedMarkers(video_frame, corners,ids) #Draw a square around the mark
        try:
            if len(ids)==1:
                c = corners[0][0]
            else:    
                c = corners[1][0]
            a,b=int(c[:, 0].mean()), int(c[:, 1].mean()) 
            print(a,b)
            cv2.circle(video_frame, (a,b), 4, (0, 0, 255), -1)   
        except Exception:
            print('error')    
        cv2.imshow('video_frame',video_frame)
        if cv2.waitKey(1) == ord('a'):
            motion_backward = LinearRelativeMotion(way.inverse())
            robot.move(motion_backward) 
            #gripper.cl()
            break

    #motion_backward = LinearRelativeMotion(way.inverse())
    #robot.move(motion_backward)''' 
    '''
    #gripper.clamp()
    way = Affine(0.35, 0.0, 0.22,0.0)
    motion_forward = LinearRelativeMotion(way)
    robot.move(motion_forward)
    gripper.open()
    # And move backwards using the inverse motion
    motion_backward = LinearRelativeMotion(way.inverse())
    robot.move(motion_backward)'''
    #gripper.open()'''
