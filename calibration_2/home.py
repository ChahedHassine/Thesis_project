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
import json
def robot_setup(dyn_rel = 0.25):
    robot.set_default_behavior()
    robot.set_EE((0.7071, -0.7071, 0.0, 0.0, -0.7071, -0.7071, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.1034, 1.0))##Gripper-Default-EE
    robot.set_load(0.73, (-0.01, 0.0, 0.03), (0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017)) #Gripper-Default-Load
    robot.set_dynamic_rel(dyn_rel)

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
    gripper = robot.get_gripper()
    gripper.gripper_force = 50.0
    # Joint motion
    joint_motion = JointMotion([-0.0046692655346566615, -0.43185659684792743, -0.00342779850873683, -2.736275297599926, 0.023197775555373598, 2.3191364502928216, 0.762515997321423])
    robot.move(joint_motion)
    gripper.open()
    # Define and move forwards
    #camera_frame = Affine(y=0.05)
    #home_pose = Affine(0.480, 0.0, 0.40)
    #robot.move(camera_frame, LinearMotion(home_pose, 1.75))
    L=[[-0.03691787134476467, 0.1945023441625456, -0.11999999999999998], [0.0006595639559028102, 0.03209259422854396, -0.11999999999999998]]
    a=json.dumps(L)
    print(a)      
    b=json.loads(a)
    print(b)
    
