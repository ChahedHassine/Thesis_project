import time
import json
import os
import pyhop 
import cv2 
class WorldModel:  # TODO: Move 2 gremlin world model with init
    """
    Stores and updates current & past world models, instances of a Pyhop State class.
    """

    def __init__(self,consumer_agent=""):
        self.agent= consumer_agent
        self.current_world_model = pyhop.State("current_world_model")
         # Planner
        if os.path.isfile('json/planner.json'):
            with open('json/planner.json') as f:
                self.current_world_model.planner = json.load(f)
        #Perception
        if os.path.isfile('json/perception.json'):
            with open('json/perception.json') as f:
                self.current_world_model.perception = json.load(f)              
        # Goals
        self.current_world_model.goals = [('move_piece','x','y')]
        self.current_world_model.piece0 = {'x':-1,'y':-1} 
        self.current_world_model.piece1= {'x':-1,'y':-1} 
        self.current_world_model.piece3= {'x':-1,'y':-1}
        self.current_world_model.piece5 = {'x':-1,'y':-1}
        self.current_world_model.perception = {"artifact1":"move_artifact@comnets-pc08","artifact2":"move_artifact2@comnets-pc08"}
        self.current_world_model.relative_position={'a':0.0,'b':0.0,'c':0.0} 

        

if __name__ == '__main__':
    # Sequence for testing
    world_model = WorldModel()
    print(world_model.current_world_model.goals)
   