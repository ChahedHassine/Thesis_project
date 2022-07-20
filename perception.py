from __future__ import print_function
from base64 import decode
import copy
import cv2
import numpy as np
import sys
import requests
import asyncio
#from sklearn.externals import joblib
import cv2
import numpy as np
import time
import json
from spade_artifact import Artifact
from loguru import logger
class Perception:
    """
    Data provided by the sensors, processed into information that update the world model. From actual world, to world
    model.  Acquires percepts via computer vision and arm controller invocation. Percept is the sensed XYZ position of
    an object in relation to the arm's frame of reference, in centimeters.
    """

    def __init__(self, init_world_model):
        self.agent=init_world_model.agent
        print("--- Initializing perception...")
        self.perception_world_model = init_world_model.current_world_model.perception
        self.artifact1=  self.perception_world_model["artifact1"]
        self.artifact2=  self.perception_world_model["artifact2"]
    def comnets(self,artifact, payload):
        if self.detect == False:
        #print('comnets**********')
        #logger.info(f"Received: [{artifact}] -> {payload}")
            contours=json.loads(payload)
            contours=tuple(np.array(x, dtype=np.int32) for x in contours )
            for cnt in contours:
                rect = cv2.minAreaRect(cnt)
                (x,y),(w,h),angle=rect
                logger.info(f"Received: [{artifact}] -> width={w},height={h}")
                self.agent.w=w
                self.agent.h=h
                break  
        else:
            pass             
    async def get_percept(self):
        print('get perception****started before if')
        if self.detect == False:
            print('get perception****started')
            await self.agent.artifacts.focus(self.artifact_1, self.comnets)
            time.sleep(60)
              

    def observe(self,input_world_model):
        return input_world_model.current_world_model.piece0 == {'x':-1,'y':-1}  and  input_world_model.current_world_model.piece1 == {'x':-1,'y':-1} and input_world_model.current_world_model.piece3 == {'x':-1,'y':-1} and  input_world_model.current_world_model.piece5 == {'x':-1,'y':-1}   

    def belief_revision(self,input_world_model,centers):
        """
        Updates the current world model: B = beliefRevisionFunction(B, ρ)
        :param input_world_model: World model, instance of the WorldModel class.
        :param percept: Dictionary.
        :return: The updated world model, instance of the WorldModel class.
        """
        try : 
            for L in centers:
                if L[-1] == float(0):
                    input_world_model.current_world_model.piece0 = {'x':L[0],'y':L[1]} 
                if L[-1] == float(1):
                    input_world_model.current_world_model.piece1 = {'x':L[0],'y':L[1]}
                if L[-1] == float(3):
                    input_world_model.current_world_model.piece3 = {'x':L[0],'y':L[1]}
                if L[-1] == float(5):
                    input_world_model.current_world_model.piece5 = {'x':L[0],'y':L[1]} 
        except Exception:
            input_world_model.current_world_model.piece0 = {'x':-1,'y':-1} 
            input_world_model.current_world_model.piece1 = {'x':-1,'y':-1}
            input_world_model.current_world_model.piece3 = {'x':-1,'y':-1}
            input_world_model.current_world_model.piece5 = {'x':-1,'y':-1}       
        return input_world_model

    def plan_revision(self,input_world_model,centers):
        """
        Updates the current world model: B = beliefRevisionFunction(B, ρ)
        :param input_world_model: World model, instance of the WorldModel class.
        :param percept: Dictionary.
        :return: The updated world model, instance of the WorldModel class.
        """
        x=False
        try : 
            for L in centers:
                if L[-1] == float(0):
                    if input_world_model.current_world_model.piece0['x']- L[0] > 0.005 or input_world_model.current_world_model.piece0['y']- L[1] > 0.005 :
                        x=True 
                        input_world_model.current_world_model.piece0 = {'x':L[0],'y':L[1]}
                        input_world_model.current_world_model.relative_position={'a':L[0],'b':L[1],'c':0.0} 
                if L[-1] == float(1):
                    if input_world_model.current_world_model.piece1['x']- L[0] >  0.005 or input_world_model.current_world_model.piece1['y']- L[1] >  0.005 :
                        x=True
                        input_world_model.current_world_model.piece1 = {'x':L[0] ,'y':L[1] }
                        input_world_model.current_world_model.relative_position={'a':L[0],'b':L[1],'c':0.0} 
                if L[-1] == float(3):
                    if input_world_model.current_world_model.piece3['x']- L[0] >  0.005 or input_world_model.current_world_model.piece3['y']- L[1] >  0.005 :
                        x=True
                        input_world_model.current_world_model.piece3 = {'x':L[0] ,'y':L[1]}
                        input_world_model.current_world_model.relative_position={'a':L[0],'b':L[1],'c':0.0} 
                if L[-1] == float(5):
                    if input_world_model.current_world_model.piece5['x']- L[0] >  0.005 or input_world_model.current_world_model.piece5['y']- L[1] >  0.005 :
                        x=True
                        input_world_model.current_world_model.piece5 = {'x':L[0] ,'y':L[1] }
                        input_world_model.current_world_model.relative_position={'a':L[0],'b':L[1],'c':0.0} 
        except Exception:
            pass        
        return input_world_model,x

if __name__ == '__main__':

    # Sequence for testing
    from world_model import WorldModel
    world_model = WorldModel()
    perception = Perception(world_model)
    while True:
        percept=perception.get_percept()
        input_world_model=perception.belief_revision(world_model,percept)
        #print(input_world_model.current_world_model.mask["user"])
        #print(input_world_model.current_world_model.video_frame["user"])
        contours=input_world_model.current_world_model.contours["user"]
        print(type(contours))
        #print(mask)
        #print(video_frame)
        cv2.imshow('video_frame',input_world_model.current_world_model.video_frame["user"])
        cv2.imshow('mask',input_world_model.current_world_model.mask["user"])  
        if cv2.waitKey(0) == ord('a'):
            break
    cv2.destroyAllWindows()

