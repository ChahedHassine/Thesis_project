import asyncio
import random
import getpass
import cv2
import numpy as np
from loguru import logger
from spade.agent import Agent
import json
from spade_artifact import Artifact, ArtifactMixin
from cv2 import aruco
image_to_arm1=np.array([[-4.39820703e-01 , 7.38561624e-02 ,-5.79022388e-02 ,-5.79022388e-02],
 [ 8.45268872e-02 , 8.69480530e-01 ,-7.54211662e-03 ,-7.54211662e-03],
 [-3.75706658e-03 , 1.11636606e-02 ,-5.92652510e-02 ,-5.92652510e-02],
 [-1.38777878e-16 , 7.21644966e-16 , 5.00000000e-01 , 5.00000000e-01]])
# cameraMatrix   
cameraMatrix1=np.array([[777.78998739,0.,696.44082619],
 [  0.,945.57159464,364.44000849],
 [  0.,0.,1.]] )
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
def calculate_XYZ(u,v,camera_matrix):
    uv_1=np.array([[u,v,1]], dtype=np.float32)
    uv_1=uv_1.T
    cam_mtx =  camera_matrix
    inverse_cam_mtx = np.linalg.inv(cam_mtx)
    XYZ=inverse_cam_mtx.dot(uv_1)
    x,y,z=XYZ[0,0],XYZ[1,0],XYZ[2,0]
    return x,y,z
class RecogniseArtifact(Artifact):
    def __init__(self, *args,id_camera: int = 0, artifact_window: str = "video-frame",cameraMatrix=cameraMatrix1,image_to_arm=image_to_arm1,**kwargs):
        super().__init__(*args, **kwargs)
        self.id_camera=id_camera
        self.artifact_window=artifact_window
        self.cameraMatrix=cameraMatrix
        self.image_to_arm=image_to_arm
    def on_available(self, jid, stanza):
        logger.success(
            "[{}] Agent {} is available.".format(self.name, jid.split("@")[0])
        )
    def on_subscribed(self, jid):
        logger.success(
            "[{}] Agent {} has accepted the subscription.".format(
                self.name, jid.split("@")[0]
            )
        )
        logger.success(
            "[{}] Contacts List: {}".format(self.name, self.presence.get_contacts())
        )
    def on_subscribe(self, jid):
        logger.success(
            "[{}] Agent {} asked for subscription. Let's aprove it.".format(
                self.name, jid.split("@")[0]
            )
        )
        self.presence.approve(jid)
        self.presence.subscribe(jid)
    async def setup(self):
        # Approve all contact requests
        self.presence.set_available()
        self.presence.on_subscribe = self.on_subscribe
        self.presence.on_subscribed = self.on_subscribed
        self.presence.on_available = self.on_available
    async def run(self):
        print("it works")
        await self.publish('[]')
        cap = cv2.VideoCapture(self.id_camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        while True: 
            ret, video_frame = cap.read()
            #print(ret)
            gray = cv2.cvtColor(video_frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters =  aruco.DetectorParameters_create()
            try:
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                frame_markers = aruco.drawDetectedMarkers(video_frame.copy(), corners, ids)
                aruco.drawDetectedMarkers(video_frame, corners,ids) #Draw a square around the mark
                #print(ids)
                stack = []
                L=[x[0] for x in ids]
                #print(L)
                for i in L:
                    if i != 2:
                        index = list(ids).index([i])
                        c = corners[index][0]
                        a,b=int(c[:, 0].mean()), int(c[:, 1].mean()) 
                        x,y,z= calculate_XYZ(a,b,self.cameraMatrix)
                        cv2.circle(video_frame, (a,b), 4, (0, 0, 255), -1)
                        pt=[x,y,z,1.0]
                        way=np.dot(self.image_to_arm, np.array(pt))[0:3]
                        L=list(x for x in way )
                        #print("Result:",L)
                        L.append(float(i))
                        #print(L)
                        if L[0]<0.16:
                            stack.append(L)
            except Exception:
                pass  
            cv2.namedWindow(self.artifact_window)         
            cv2.imshow(self.artifact_window,video_frame)
            print(stack)
            cv2.waitKey(1)  
            await self.publish(json.dumps(stack))
            logger.info("published")
            #await asyncio.sleep(0.5)
            
       
if __name__ == "__main__":  
    #artifact_jid = "hassine@desktop-ahuhkk8"
    artifact_jid ="move_artifact@comnets-pc08"
    artifact_passwd = "admin"
    artifact = RecogniseArtifact(artifact_jid, artifact_passwd,id_camera=0)
    future = artifact.start()
    future.result()
    artifact.join()
    '''
    artifact_jid ="move_artifact2@comnets-pc08"
    artifact_passwd = "admin"
    artifact2 = RecogniseArtifact(artifact_jid, artifact_passwd,id_camera=2,artifact_window='video_2')
    future = artifact2.start()
    future.result()
    artifact2.join()'''