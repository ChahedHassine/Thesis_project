from re import A
import pyhop
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

class HierarchicalTaskNetworkPlanner:
    """
    Automated planner for hierarchical tasks (with pre-conditions, sub-tasks and effects).
    Deterministic, with backtracking search. Elements:
    1. Compound tasks (Methods): Compositions of simpler tasks.
    2. Primitive tasks (Operators): Base tasks.
    3. Helper methods.
    """
    def __init__(self, init_world_model):
        self.failure_reason = ""
        #self.planner_world_model = init_world_model.current_world_model.planner
        #self.verbose = self.planner_world_model["verbose"]
    # Methods (compound tasks)
    def locate_piece(state,a,b):
            return state
    def move_arm_down1(state,a,b) :
            return state 
    def gripper_clamp(state,a,b) :
            return state                     
    def move_arm_up(state,a,b) :
            return state
    def initial_position(state,a,b) :
            return state   
    def move_basket(state,a,b) :
            return state                     
    def move_arm_down2(state,a,b) :
            return state 
    def gripper_open(state,a,b) :
            return state 
    def move_chess(state,a,b) :
            return state  
    def move_arm_down3(state,a,b) :
            return state                                
    pyhop.declare_operators(locate_piece,move_arm_down1,gripper_clamp,move_arm_up,initial_position,move_basket,move_arm_down2,gripper_open,move_chess,move_arm_down3)
    def move_arm(state,a,b):
        if state.piece0[a] != -1 and state.piece0 [b] != -1 :
            #print('callback to recognize object')
            if ontology(0) == True:
                return [('initial_position',a,b),('gripper_open',a,b),('locate_piece',a,b),('move_arm_down1',a,b),('gripper_clamp',a,b),('move_arm_up',a,b),('initial_position',a,b),('move_basket',a,b),('move_arm_down2',a,b),('gripper_open',a,b),('move_arm_up',a,b),('initial_position',a,b)]
            else:
                return   [('initial_position',a,b),('gripper_open',a,b),('locate_piece',a,b),('move_arm_down1',a,b),('gripper_clamp',a,b),('move_arm_up',a,b),('initial_position',a,b),('move_chess',a,b),('move_arm_down3',a,b),('gripper_open',a,b),('move_arm_up',a,b),('initial_position',a,b)]  
        if  state.piece1[a] != -1 and state.piece1 [b] != -1 :
            if ontology(1) == True:
                return [('initial_position',a,b),('gripper_open',a,b),('locate_piece',a,b),('move_arm_down1',a,b),('gripper_clamp',a,b),('move_arm_up',a,b),('initial_position',a,b),('move_basket',a,b),('move_arm_down2',a,b),('gripper_open',a,b),('move_arm_up',a,b),('initial_position',a,b)]
            else:
                return  [('initial_position',a,b),('gripper_open',a,b),('locate_piece',a,b),('move_arm_down1',a,b),('gripper_clamp',a,b),('move_arm_up',a,b),('initial_position',a,b),('move_chess',a,b),('move_arm_down3',a,b),('gripper_open',a,b),('move_arm_up',a,b),('initial_position',a,b)] 
        if  state.piece3[a] != -1 and state.piece3 [b] != -1 :    
            if ontology(3) == True:
                return [('initial_position',a,b),('gripper_open',a,b),('locate_piece',a,b),('move_arm_down1',a,b),('gripper_clamp',a,b),('move_arm_up',a,b),('initial_position',a,b),('move_basket',a,b),('move_arm_down2',a,b),('gripper_open',a,b),('move_arm_up',a,b),('initial_position',a,b)]
            else:
                return  [('initial_position',a,b),('gripper_open',a,b),('locate_piece',a,b),('move_arm_down1',a,b),('gripper_clamp',a,b),('move_arm_up',a,b),('initial_position',a,b),('move_chess',a,b),('move_arm_down3',a,b),('gripper_open',a,b),('move_arm_up',a,b),('initial_position',a,b)] 
        if  state.piece5[a] != -1 and state.piece5 [b] != -1 :    
            if ontology(5) == True:
                return [('initial_position',a,b),('gripper_open',a,b),('locate_piece',a,b),('move_arm_down1',a,b),('gripper_clamp',a,b),('move_arm_up',a,b),('initial_position',a,b),('move_basket',a,b),('move_arm_down2',a,b),('gripper_open',a,b),('move_arm_up',a,b),('initial_position',a,b)]
            else:
                return   [('initial_position',a,b),('gripper_open',a,b),('locate_piece',a,b),('move_arm_down1',a,b),('gripper_clamp',a,b),('move_arm_up',a,b),('initial_position',a,b),('move_chess',a,b),('move_arm_down3',a,b),('gripper_open',a,b),('move_arm_up',a,b),('initial_position',a,b)]             
    
    #TODO: nesting of more than 2 levels deep
    pyhop.declare_methods('move_piece',move_arm) # declare methods


    def get_plans(self, world_model, goal):
        """
        Returns all the suggested plans, given a world model (state object), goal (list) and an internal collection of
        primitive and compound tasks.
        :param world_model: The current perceived world state (json object).
        :param goal: The end goal we try to achieve (tuple).
        :return: List of suggested plans.
        """

        if goal != "":
            return pyhop.pyhop(world_model, goal, verbose=3, all_plans=True, sort_asc=True)
        else:
            return ""

if __name__ == '__main__':

    end_goal = [('move_piece','x','y')]
    intentions = end_goal  # I := I0; Initial Intentions
    from world_model import WorldModel
    beliefs = WorldModel()  # B := B0; Initial Beliefs
    htn_planner = HierarchicalTaskNetworkPlanner(beliefs)
    print(beliefs.current_world_model.piece0)
    plans =htn_planner.get_plans(beliefs.current_world_model,intentions)  # π := plan(B, I); MEANS_END REASONING
    print(plans)
    