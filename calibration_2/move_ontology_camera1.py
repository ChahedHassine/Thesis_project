from re import L
import numpy as np
from argparse import ArgumentParser

from frankx import Robot, RobotMode, Affine, MotionData, Reaction, Measure, JointMotion, ImpedanceMotion, StopMotion, PathMotion, LinearMotion, WaypointMotion, Waypoint, LinearRelativeMotion
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
from rdflib import Graph
from rdflib.namespace import CSVW, DC, DCAT, DCTERMS, DOAP, FOAF, ODRL2, ORG, OWL, PROF, PROV, RDF, RDFS, SDO, SH, SKOS, SOSA, SSN, TIME, VOID, XMLNS, XSD

from rdflib import URIRef

g = Graph()

g.parse("robot.ntriples")
def ontology(id=1):
    g = Graph()
    g.parse("robot.ntriples")
    bob="http://example.org/piece"+str(id)
    bob = URIRef(bob)
    #print(bob)
    group2=URIRef("http://example.org/group2")
    if (bob, RDF.type, group2) in g:
        return True
    else:
        return False   

image_to_arm=np.array([[-4.39820703e-01 , 7.38561624e-02 ,-5.79022388e-02 ,-5.79022388e-02],
 [ 8.45268872e-02 , 8.69480530e-01 ,-7.54211662e-03 ,-7.54211662e-03],
 [-3.75706658e-03 , 1.11636606e-02 ,-5.92652510e-02 ,-5.92652510e-02],
 [-1.38777878e-16 , 7.21644966e-16 , 5.00000000e-01 , 5.00000000e-01]])
# cameraMatrix   
cameraMatrix=np.array([[777.78998739,0.,696.44082619],
 [  0.,945.57159464,364.44000849],
 [  0.,0.,1.]] )
def calculate_XYZ(u,v,camera_matrix):
    uv_1=np.array([[u,v,1]], dtype=np.float32)
    uv_1=uv_1.T
    cam_mtx =  camera_matrix
    inverse_cam_mtx = np.linalg.inv(cam_mtx)
    XYZ=inverse_cam_mtx.dot(uv_1)
    x,y,z=XYZ[0,0],XYZ[1,0],XYZ[2,0]
    return x,y,z
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
def move_robot(L,x):
    way = Affine(L[0],L[1],0.0 )
    motion_forward = LinearRelativeMotion(way)
    robot.move(motion_forward)
    recover_error(motion_forward)    
    #robot.set_dynamic_rel(0.01)
    way = Affine(0.0,0.0,-0.14 )
    motion_forward = LinearRelativeMotion(way)
    robot.move(motion_forward)
    recover_error(motion_forward)
    gripper.clamp(0.04)   
    recover_error()  
    way = Affine(0.0,0.0,0.17 )
    motion_forward = LinearRelativeMotion(way)
    robot.move(motion_forward)
    recover_error(motion_forward)
    joint_motion = JointMotion([-0.0046692655346566615, -0.43185659684792743, -0.00342779850873683, -2.736275297599926, 0.023197775555373598, 2.3191364502928216, 0.762515997321423])
    robot.move(joint_motion)
    gripper.open()
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
    robot.velocity_rel = 0.2
    robot.acceleration_rel = 0.05
    robot.jerk_rel = 0.001
    gripper = robot.get_gripper()
    gripper.gripper_force = 50.0
    
    #joint_motion = JointMotion([-2.4417100295260634, 1.0681055992583837, 2.2147164654894858, -2.893747881959243, -0.8530632804061193, 1.936590144773569, 1.1926808407695957])
    joint_motion = JointMotion([-0.0046692655346566615, -0.43185659684792743, -0.00342779850873683, -2.736275297599926, 0.023197775555373598, 2.3191364502928216, 0.762515997321423])
    robot.move(joint_motion)
    gripper.open()
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
        print(ids)
        try:
            L=[x[0] for x in ids]
            #print(L)
            stack = []
            for i in L:
                index = list(ids).index([i])
                c = corners[index][0]
                a,b=int(c[:, 0].mean()), int(c[:, 1].mean()) 
                if ontology(i) == True :
                    x,y,z= calculate_XYZ(a,b,cameraMatrix)
                    cv2.circle(video_frame, (a,b), 4, (0, 0, 255), -1)
                    pt=[x,y,z,1.0]
                    way=np.dot(image_to_arm, np.array(pt))[0:3]
                    L=list(x for x in way )
                    print("Result:",L) 
                    stack.append(L)
            cv2.imshow('video_frame',video_frame)    
            if cv2.waitKey(1) == ord('a'):
                x=0.0
                for L in stack:
                    move_robot(L,x)
                    x=x+0.005
                    break
        except Exception:
            pass        