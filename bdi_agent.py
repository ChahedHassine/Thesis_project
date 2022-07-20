import time
from spade.agent import Agent
from spade.behaviour import OneShotBehaviour
from spade.message import Message
from spade.template import Template
from spade.behaviour import CyclicBehaviour,OneShotBehaviour,PeriodicBehaviour
#from object_detector import *
import numpy as np
import json
from world_model import WorldModel
from hierarchical_task_network_planner import HierarchicalTaskNetworkPlanner
from collections import deque
from perception import Perception
from spade_artifact import Artifact, ArtifactMixin
from loguru import logger
import asyncio
import cv2
from argparse import ArgumentParser
from frankx import Robot, RobotMode, Affine, MotionData, Reaction, Measure, JointMotion, ImpedanceMotion, StopMotion, PathMotion, LinearMotion, WaypointMotion, Waypoint, LinearRelativeMotion
from coordination import Coordination
import datetime
from rdflib import Graph
from rdflib.namespace import CSVW, DC, DCAT, DCTERMS, DOAP, FOAF, ODRL2, ORG, OWL, PROF, PROV, RDF, RDFS, SDO, SH, SKOS, SOSA, SSN, TIME, VOID, XMLNS, XSD
from rdflib import URIRef
def camera_ontology():
    #search in web ontology for another artifact
    g = Graph()
    g.parse("robot.ntriples")
    camera=URIRef("http://example.org/camera") #The existing cameras belong to the camera ontology class
    for c in g.subjects(RDF.type, camera):
        if c.split('://')[1] != "move_artifact@comnets-pc08":#If the camera serached in the ontology has an URI different to the used camera then use it
                #print(c.split('://')[1])
                return c.split('://')[1] 
class MyBehav(OneShotBehaviour):
        def __init__(self, consumer_agent,robot, gripper):
                super().__init__()
                self.agent=consumer_agent #attribut agent that subscribe to different artifacts
                self.robot=robot #attribut robot that controls the robot
                self.gripper=gripper #attribut robot that controls the robot
        def comnets(self,artifact, payload): #function that update the positions of the pieces
            logger.info(f"Received: [{artifact}] -> {payload}") #show that the agent receive the notfifications from the artifacts
            centers=json.loads(payload)
            self.centers= centers   #update the positions of pieces with the centers list         
            #print(self.centers)
        async def on_start(self):
            print("Starting behaviour . . .")
            self.terminate = False
            self.SUCCESS = False
            self.verbose = True # if you don't need to show any thing than verbose == False
            #self.counter = 0
            self.centers=[] # The centers list is empty at the beginning
            # Initialization
            self.beliefs = WorldModel(self.agent)  # B := B0; Initial Beliefs
            self.goals = self.beliefs.current_world_model.goals # Set initial goals
            self.intentions = self.beliefs.current_world_model.goals  # I := I0; Initial Intentions
            self.htn_planner = HierarchicalTaskNetworkPlanner(self.beliefs) #initiate the htn_planner 
            self.perception = Perception(self.beliefs) #initiate the perception class to observe the environment
            self.coordination = Coordination(self.beliefs,self.robot,self.gripper,self.agent) #initiate the coordination class to observe the environment
            #self.what, self.why, self.how_well, self.what_else, self.why_failed = "", "", "", "", ""
            self.plans = [] #plan is empty at the beginning
            self.selected_plan = [] #selected_plan is empty at the beginning
            self.percept = {}
            self.action = ""  # no action  at the beginning
            await asyncio.sleep(1)
        async def run(self):
            '''self.counter += 1
            if self.counter > 3:
                self.kill(exit_code=10)'''
            #print("work before the loop")
            while True: #for loop running
                if not self.SUCCESS and not self.terminate :
                #print("enter the first if")
                    if len(self.selected_plan) == 0: #if no plan found
                        y=0 #y shift for the pieces
                        x=0 #x shift for the pieces
                        time.sleep(4)
                        print("enter the second if")
                        #print(self.perception.artifact)
                        #await self.agent.artifacts.focus(self.perception.artifact, self.comnets)# get next percept ρ; OBSERVE the world
                        #await asyncio.sleep(1) 
                        self.observe = self.perception.observe(self.beliefs) #check if no piece is found
                        if  self.observe:  #If true than subscribe to the first artifact
                            await self.agent.artifacts.focus(self.perception.artifact1, self.comnets)# get next percept ρ; OBSERVE the world
                            if self.centers ==[]: #if the first camera don't find pieces than search in the ontology for another one
                                    artifact=camera_ontology()
                                    await self.agent.artifacts.focus(artifact, self.comnets)
                            self.beliefs = self.perception.belief_revision(self.beliefs,self.centers)  # B:= brf(B, ρ); #update each piece position
                        #print(self.beliefs.current_world_model.piece0)
                        self.intentions = self.deliberate(self.beliefs,self.intentions)  # DELIBERATE about what INTENTION to achieve next
                        print(self.intentions)
                        self.SUCCESS = True if self.intentions == "" else False
                        #self.plans = self.htn_planner.get_plans(self.beliefs.current_world_model,self.intentions)  # π := plan(B, I); MEANS_END REASONING
                        try:
                            self.plans = self.htn_planner.get_plans(self.beliefs.current_world_model,self.intentions)  # π := plan(B, I); MEANS_END REASONING #get plans for acheiving the goal
                        except Exception:
                            print('no plan selected')
                            self.plans=False    
                        #print(self.plans)
                        if self.plans != False:
                            if len(self.plans) > 0:
                                print("works good........")
                                self.plans.sort(key=len)  # TODO: check why sorting doesn't work on "deeper" levels
                                print(self.plans)
                                if self.verbose:
                                    print("Plan: {}".format(self.plans[0]))
                                self.selected_plan = deque(self.plans[0])  # TODO: Use a "cost function" to evaluate the best plan, not shortest
                                #self.why_failed = ""
                                #print(len(self.selected_plan))
                                #print(self.selected_plan)
                        else:
                            print("failure no plan")
                    else:
                        '''if plan selected than execute actions'''
                        print("yessssssssssssssssssssss enter the second condition")
                        print(self.selected_plan)
                        self.action, self.selected_plan = self.selected_plan.popleft(), self.selected_plan  # α := hd(π); π := tail(π); pop the selected plan and get the action
                        if self.verbose:
                            print("Action: {}".format( self.action))  
                        self.beliefs,x,y=self.coordination.execute_action(self.action, self.beliefs,x,y)  # execute(α); execute the action

        def deliberate(self, current_beliefs, current_intentions):
            """
            Decide WHAT sate of affairs you want to achieve.
            :param current_beliefs: WordModel instance, the world model, information about the world.
            :param current_intentions: List of tuples, tasks that the agent has COMMITTED to accomplish.
            :return: List of tuples, filtered intentions that the agent COMMITS to accomplish.
            """
            # 1. Option Generation: The agent generates a set of possible alternatives.
            current_desires = current_intentions  # TODO: D := option(B, I);

            # 2. Filtering: Agent chooses between competing alternatives and commits to achieving them.
            current_intentions = self.filter_intentions(
                current_beliefs, current_desires, current_intentions)  # I := filter(B, D, I);

            return current_intentions

        def filter_intentions(self, current_beliefs, current_desires, current_intentions):
            """
            Choose between competing alternatives and COMMITTING to achieving them.
            :param current_beliefs: WordModel instance, the world model, information about the world.
            :param current_desires: List of tuples, tasks that the agent would like to accomplish.
            :param current_intentions: List of tuples, tasks that the agent has COMMITTED to accomplish.
            :return: List of tuples, filtered intentions that the agent COMMITS to accomplish.
            """
            for current_intention in current_intentions:
                '''if current_intention == self.goals[0] : 
                    current_intentions = "come to an end"  # if goal(s) achieved, empty I'''
            return current_intentions



class ReceiverAgent(ArtifactMixin, Agent):             
    async def setup(self):
        logger.info("Agent ready")
        self.presence.approve_all = True
        self.presence.set_available()
        self.presence.subscribe("move_artifact@comnets-pc08")
        # Connect to the robot
        global robot, gripper, is_master_mode
        parser = ArgumentParser()
        parser.add_argument("--robotip", default="192.168.1.2", help="robot ip")
        parser.add_argument("--gatewayip", default="192.168.2.1", help="gateway ip")
        args = parser.parse_args()
        robot = Robot(fci_ip = args.robotip, repeat_on_error = False)
        robot.set_default_behavior()
        robot.recover_from_errors()
        # Reduce the acceleration and velocity dynamic
        robot.velocity_rel = 0.2
        robot.acceleration_rel = 0.05
        robot.jerk_rel = 0.001
        # set the gripper parameters
        gripper = robot.get_gripper()
        gripper.gripper_force = 50.0
        #initiate the behaviour of the agent with the robot and the gripper 
        self.my_behav = MyBehav(self,robot, gripper)
        self.add_behaviour(self.my_behav)
        


if __name__ == "__main__":
    receiveragent = ReceiverAgent("hassine@comnets-pc08", "admin")
    future = receiveragent.start()
    future.result() # wait for receiver agent to be prepared.
    while receiveragent.is_alive():
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            receiveragent.stop()
            break