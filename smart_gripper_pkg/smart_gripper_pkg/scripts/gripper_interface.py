import sys
import os
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

class Gripper:
    def __init__(self, router, proportional_gain = 1.0, sleep_time=0.5):

        self.sleep_time = sleep_time
        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

        self.gripper_command = Base_pb2.GripperCommand()
        self.finger = self.gripper_command.gripper.finger.add()

    def position(self):        
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
        position = gripper_measure.finger[0].value  
        return position

        
    def width(self,width):
       GripperRequest """
        width: type float, range value between (0-1)
        """
        self.gripper_command.mode = Base_pb2.GRIPPER_POSITION
        #position = 1 - width / 140
        position = 0.0 if width < 0.0 else 1 if width > 1.0 else width

        self.finger.value = position
        self.base.SendGripperCommand(self.gripper_command)
        time.sleep(self.sleep_time)

    def vel(self, vel):
        #print ("Controlling gripper using speed command...")
        self.gripper_command.mode = Base_pb2.GRIPPER_SPEED
        self.finger.value = vel
        self.base.SendGripperCommand(self.gripper_command)
        
        # Wait for reported speed to be 0
        gripper_pose_request = Base_pb2.GripperRequest()
        gripper_pose_request.mode = Base_pb2.GRIPPER_POSITION

        gripper_vel_request = Base_pb2.GripperRequest()
        gripper_vel_request.mode = Base_pb2.GRIPPER_SPEED
        speed_percentage_error = 0.001
        
        #while True:
        #    self.finger.value = vel
        #    self.base.SendGripperCommand(self.gripper_command) 
        
        while True:
            gripper_vel_measure = self.base.GetMeasuredGripperMovement(gripper_vel_request)
            gripper_pose_measure = self.base.GetMeasuredGripperMovement(gripper_pose_request)
            #print("debug")
            if len (gripper_vel_measure.finger):
                #print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                
                diff_speed = abs(abs(vel)-gripper_vel_measure.finger[0].value)/vel
                #print(f"Target Vel: {vel}, \t Current Vel: {gripper_vel_measure.finger[0].value} \t Diff_speed: {diff_speed}")
                
                # If target vel is not enough to make the gripper move
                #if abs(vel) < 0.009:
                #    #print("Not enough vel to make gripper move. Setting to min: 0.01")
                #    if vel < 0:
                #        self.finger.value = -0.009
                #    else:
                #        self.finger.value = 0.009
                #    self.base.SendGripperCommand(self.gripper_command)
                #    break
                # If achieved speed is within the range
                if diff_speed <= speed_percentage_error:
                    self.finger.value = 0
                    self.base.SendGripperCommand(self.gripper_command)
                    
                    #print("Exiting command, vel achieved")
                    break
                # If Gripper is completely openned 
                elif gripper_pose_measure.finger[0].value < 0.01:
                    # We can assume when calling SendGripperCommand internally
                    # has some flags that verifies if the gripper is completely 
                    # open/close and cant keep moving thus is not necesseary to 
                    # add:
                    #    self.finger.value = 0
                    #    self.base.SendGripperCommand(self.gripper_command)
                    
                    #print("Gripper completely openned")
                    break
                # If Gripper is completely closed
                elif gripper_pose_measure.finger[0].value >= 9.98:
                    # We can assume when calling SendGripperCommand internally
                    # has some flags that verifies if the gripper is completely 
                    # open/close and cant keep moving thus is not necesseary to 
                    # add:
                    #    self.finger.value = 0
                    #    self.base.SendGripperCommand(self.gripper_command)
                    
                    #print("Gripper completely closed")
                    break
                # There is a 3 case where the finger cant keep
                # moving beacuse of an object. Probably the internal
                # command handles that case with force detection
                # so no need to implemented ourselves.
            else: # Else, no finger present in answer, end loop
                break
    

def main():
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        example = Gripper(router)
        example.gripper_width(0.545)        # example.gripper_vel(-0.1)

if __name__ == "__main__":
    main()