from frankx import Robot, RobotMode, Affine, MotionData, Reaction, Measure, JointMotion, ImpedanceMotion, StopMotion, PathMotion, LinearMotion, WaypointMotion, Waypoint, LinearRelativeMotion
import time
def recover_error(robot,motion_forward=''):
        if robot.read_once().robot_mode == RobotMode.Reflex :
            print("recover_from_errors")
            robot.recover_from_errors()
            time.sleep(0.5)
            if  motion_forward != '':
                robot.move(motion_forward)
                print('moving....')
                if robot.read_once().robot_mode == RobotMode.Reflex :
                    recover_error(robot,motion_forward)
                    time.sleep(0.5)
            time.sleep(0.5)

def recover_error_locate(robot,r,motion_forward=''):
        if robot.read_once().robot_mode == RobotMode.Reflex :
            print("recover_from_errors")
            robot.recover_from_errors()
            time.sleep(0.5)
            if  motion_forward != '':
                robot.move(r,motion_forward)
                print('moving....')
                if robot.read_once().robot_mode == RobotMode.Reflex :
                    recover_error(robot,r,motion_forward)
                    time.sleep(0.5)
            time.sleep(0.5)
class Coordination:
    """
    Executes actions (discrete), by invoking control commands (continuous).
    """

    def __init__(self, world_model,robot,gripper,agent):
        self.verbose = False
        self.world_model=world_model
        self.robot=robot
        self.gripper=gripper
        self.agent=agent
    def execute_action(self, action, world_model,x,y):
        """
        Executes actions by using information from the world model and invoking control commands.
        """ 
        if action == ('locate_piece', 'x', 'y'):
            if world_model.current_world_model.piece0['x'] != -1:#if the piece 0 is detected
                    L=[world_model.current_world_model.piece0['x'],world_model.current_world_model.piece0['y'],0.0] #update the position list
                    y=0.0
                    x=0.04
            elif world_model.current_world_model.piece1['x'] != -1:#if the piece 1 is detected
                    L=[world_model.current_world_model.piece1['x'],world_model.current_world_model.piece1['y'],0.0]  #update the position list
                    y=0.0
                    x=-0.04
            elif world_model.current_world_model.piece3['x'] != -1: #if the piece 3 is detected
                    L=[world_model.current_world_model.piece3['x'],world_model.current_world_model.piece3['y'],0.0] #update the position list
                    y=0.07 
                    x=-0.03
            elif world_model.current_world_model.piece5['x'] != -1: #if the piece 5 is detected
                    L=[world_model.current_world_model.piece5['x'],world_model.current_world_model.piece5['y'],0.0]   #update the position list
                    y=0.07  
                    x=0.04       
            way = Affine(L[0],L[1],0.0 ) 
            motion_forward = LinearRelativeMotion(way)
            self.robot.move(motion_forward) # move the robot to the piece
            recover_error(self.robot,motion_forward) #recover from errors
        elif action == ('move_arm_down1', 'x', 'y'):
            way = Affine(0.0,0.0,-0.14 )
            motion_forward = LinearRelativeMotion(way)
            self.robot.move(motion_forward) # move the robot down 
            recover_error(self.robot,motion_forward) #recover from errors
            #time.sleep(2)
        elif action == ('gripper_clamp', 'x', 'y'): # clamp the piece and update the position of the detected piece
            if world_model.current_world_model.piece0['x'] != -1:
                    world_model.current_world_model.piece0['x']=-1
                    world_model.current_world_model.piece0['y']=-1
            elif world_model.current_world_model.piece1['x'] != -1:
                    world_model.current_world_model.piece1['x']=-1
                    world_model.current_world_model.piece1['y']=-1
            elif world_model.current_world_model.piece3['x'] != -1:
                    world_model.current_world_model.piece3['x']=-1
                    world_model.current_world_model.piece3['y']=-1
            elif world_model.current_world_model.piece5['x'] != -1:
                    world_model.current_world_model.piece5['x']=-1
                    world_model.current_world_model.piece5['y']=-1  
            self.gripper.clamp(0.04)


        elif action == ('move_arm_up', 'x', 'y'):
            way = Affine(0.0,0.0,0.15 )
            motion_forward = LinearRelativeMotion(way) # move the robot up 
            self.robot.move(motion_forward)  #recover from errors
            recover_error(self.robot,motion_forward)
            #time.sleep(2)
        elif action == ('initial_position', 'x', 'y'): # move the robot to the initial position 
            joint_motion = JointMotion([-0.0046692655346566615, -0.43185659684792743, -0.00342779850873683, -2.736275297599926, 0.023197775555373598, 2.3191364502928216, 0.762515997321423])
            self.robot.move(joint_motion)
            recover_error(self.robot,joint_motion)
            world_model.current_world_model.relative_position={'a':0.0,'b':0.0,'c':0.0} 
            #time.sleep(2)
        elif action == ('move_basket', 'x', 'y'): # move the robot to the basket
            way = Affine(0.24-x, -0.1+y, 0.05,0.0)
            motion_forward = LinearRelativeMotion(way)
            self.robot.move(motion_forward)
            recover_error(self.robot,motion_forward)
            #time.sleep(2)
        elif action == ('move_chess', 'x', 'y'): # move the robot to the chess
            way = Affine(-0.25+x, 0.29+y, 0.05,0.0)
            motion_forward = LinearRelativeMotion(way)
            self.robot.move(motion_forward)
            recover_error(self.robot,motion_forward)
            #time.sleep(2)
        elif action == ('move_arm_down2', 'x', 'y'): # move the robot down
            way = Affine(0.0, 0.0, -0.17,0.0)
            motion_forward = LinearRelativeMotion(way)
            self.robot.move(motion_forward)
            recover_error(self.robot,motion_forward)
            #time.sleep(2)
        elif action == ('gripper_open', 'x', 'y'): # open the gripper
            self.gripper.open()
            #time.sleep(2)
        elif action == ('move_arm_down3', 'x', 'y'):
            way = Affine(0.0, 0.0, -0.2,0.0)
            motion_forward = LinearRelativeMotion(way)
            self.robot.move(motion_forward)
            recover_error(self.robot,motion_forward)   
        return world_model,x,y

if __name__ == '__main__':

    # Sequence for testing
    from world_model import WorldModel
    current_world_model = WorldModel()
    coordination = Coordination(current_world_model)
    coordination.control.control_world_model["send_requests"] = False
    coordination.control.control_world_model["center_init"] = False
    coordination.control.control_world_model["detect_last_position"] = False
    coordination.execute_action(('initialize', 'arm'), current_world_model.current_world_model)
    coordination.execute_action(('open_hand', ), current_world_model.current_world_model)
    coordination.execute_action(('move_arm_above', 'target_object'), current_world_model.current_world_model)
    coordination.execute_action(('move_arm', 'target_object'), current_world_model.current_world_model)
    coordination.execute_action(('close_hand',), current_world_model.current_world_model)
    coordination.execute_action(('move_arm_up', 'target_object'), current_world_model.current_world_model)
    coordination.execute_action(('move_arm_above', 'container'), current_world_model.current_world_model)
    coordination.execute_action(('open_hand', ), current_world_model.current_world_model)
    import numpy as np
    target_position = np.array([20, -20.0, 20]) * coordination.control.control_world_model["scale"]
    last_servo_values = current_world_model.current_world_model.location["servo_values"]
    action_successful_test = coordination.control.move_arm(np.array(target_position), last_servo_values)