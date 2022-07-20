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




image_to_arm=np.array([[-3.26119036e+00 , 1.68067199e+00 , 1.27566016e-02  ,1.27566016e-02],
 [ 2.63740785e+00 , 1.97309011e+00 ,-2.29012727e-03 ,-2.29012727e-03],
 [-1.64744823e-01,  1.60628065e-01 ,-4.87132832e-02 ,-4.87132832e-02],
 [-7.99360578e-15 ,-1.06581410e-14 , 5.00000000e-01 , 5.00000000e-01]])


'''image_to_arm=np.array([[-3.20787773e+00 , 1.95892785e+00 , 5.23016664e-03 , 5.23016664e-03],
 [ 2.78783829e+00 , 1.88199282e+00 ,-2.00585483e-03 ,-2.00585483e-03],
 [ 2.75329224e-02 , 1.48683451e-04, -5.91563923e-02 ,-5.91563923e-02],
 [ 3.33066907e-16 ,-7.99360578e-15 , 5.00000000e-01 , 5.00000000e-01]])'''

'''image_to_arm=np.array([[-3.17243035e+00 , 1.85372653e+00 , 3.59950551e-03 , 3.59950551e-03],
 [ 2.64923466e+00 , 1.82795508e+00 ,-1.14064209e-03 ,-1.14064209e-03],
 [ 3.17197717e-02 , 8.02535513e-02 ,-5.98181602e-02, -5.98181602e-02],
 [ 2.16493490e-15 , 2.22044605e-15 , 5.00000000e-01 , 5.00000000e-01]])'''

'''image_to_arm=np.array([[-3.75990727e+00 , 1.52073601e+00 , 1.01181805e-02 , 1.01181805e-02],
 [ 2.79908419e+00 , 2.02461518e+00 ,-1.97240271e-03, -1.97240271e-03],
 [-1.53357773e-01, -6.58197791e-02 ,-4.80704545e-02 ,-4.80704545e-02],
 [ 6.21724894e-15 , 6.43929354e-15 , 5.00000000e-01 , 5.00000000e-01]])'''

'''image_to_arm=np.array([[-3.44135382e+00 , 1.93010480e+00 , 1.01549024e-02 , 1.01549024e-02],
 [ 2.28589279e+00 , 1.96235465e+00 ,-3.42749645e-03 ,-3.42749645e-03],
 [-3.43380332e-01 , 4.31792203e-02 ,-4.59840989e-02 ,-4.59840989e-02],
 [ 1.82076576e-14 ,-4.88498131e-15 , 5.00000000e-01 , 5.00000000e-01]])'''

'''image_to_arm=np.array([[-3.44528412e+00 , 1.57430689e+00 , 1.41962433e-02 , 1.41962433e-02],
 [ 2.73847958e+00  ,1.72908723e+00 ,-5.73033702e-04 ,-5.73033702e-04],
 [ 1.08382081e-16 ,-8.22187756e-16 ,-6.00000000e-02, -6.00000000e-02],
 [ 3.55271368e-15 , 7.10542736e-15 , 5.00000000e-01 , 5.00000000e-01]])'''

# cameraMatrix   
cameraMatrix = np.array([[1.09706045e+04,0.00000000e+00,5.22869579e+02],
    [0.00000000e+00,4.94207851e+03,2.70272151e+02],
    [0.00000000e+00,0.00000000e+00,1.00000000e+00]])
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
    cap = cv2.VideoCapture(2)
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