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
        """
        width: type float, range value between (0-1)
        """
        self.gripper_command.mode = Base_pb2.GRIPPER_POSITION
        #position = 1 - width / 140
        position = 0.0 if width < 0.0 else 1 if width > 1.0 else width

        self.finger.value = position
        self.base.SendGripperCommand(self.gripper_command)
        time.sleep(self.sleep_time)

    def vel(self,vel):
        #print ("Controlling gripper using speed command...")
        self.gripper_command.mode = Base_pb2.GRIPPER_SPEED
        self.finger.value = vel
        self.base.SendGripperCommand(self.gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported speed to be 0
        #gripper_request.mode = Base_pb2.GRIPPER_SPEED
        #while True:
        #    gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
        #    if len (gripper_measure.finger):
        #        #print("Current speed is : {0}".format(gripper_measure.finger[0].value))
        #        if gripper_measure.finger[0].value == 0.0:
        #            break
        #    else: # Else, no finger present in answer, end loop
        #        break
    

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