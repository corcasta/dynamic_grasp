#!/usr/bin/env python3
import time
import math
# We are assuming we have access to the following classes
from .gripper_interface import Gripper
from tactile_sensor_pkg.scripts.tactile_force import TactileForce


class PID():
    def __init__(self, kp, kd, ki):
        """
        kp: type float, CONTROLLER GAINS
        ki: type float, CONTROLLER GAINS
        kd: type float, CONTROLLER GAINS
        """
        self.accum = 0.0
        self.prev_time = 0.0
        self.delta_time = 0.0
        self.prev_error = 0.0
        
        if kp:
            self.kp = kp
        else:
            self.kp = 1.0

        if ki:
            self.ki = ki
        else:
            self.ki = 0.0

        if kd:
            self.kd = kd
        else:
            self.kd = 0.1


    def update_time_variables(self):
        self.delta_time = time.time() - self.prev_time
        self.prev_time += self.delta_time


    def update_prev_error(self, error):
        self.prev_error = error


    def update_gains(self, kp, kd, ki):
        """
        kp: type float, CONTROLLER GAINS
        ki: type float, CONTROLLER GAINS
        kd: type float, CONTROLLER GAINS
        """
        if kp:
            self.kp = kp
        if ki:
            self.ki = ki
        if kd:
            self.kd = kd


    def proportional_block(self, error):
        return self.kp*error


    def integral_block(self, error):
        self.accum += self.ki*(error*self.delta_time)
        return self.accum


    def derivative_block(self, error):
        return self.kd * (error - self.prev_error)/self.delta_time
    
    
    def forward(self, error):
        # MANDATORY to call first "update_time_variables" before calling PID blocks
        self.update_time_variables()
        
        pid_output = self.proportional_block(error) + self.integral_block(error) + self.derivative_block(error)

        # MANDATORY to call "update_prev_error" after calculating PID output
        self.update_prev_error(error)

        return pid_output

        
class ForceController():
    def __init__(self, router, camera_id, tactile_threshold, sleep_time, friction_coeff, max_output_val, min_output_val, gains):
        """
        router:             type ....
        camera_id:          type int
        tactile_threshold:  type int
        sleep_time:         type float (value between 0.1 - 1.0)
        friction_coeff:     type float
        max_output_val:     type float
        gains:              type dict, {"kp": float, "ki": float, "kd":float}
        """
        self.friction_coeff = friction_coeff
        self.max_output_val = max_output_val
        self.min_output_val = min_output_val  
        self.force_x_filter = []
        self.force_y_filter = []
        self.filter = []

        self.pid_block = PID(**gains)
        self.gripper_block = Gripper(router, sleep_time=sleep_time)

        # WE NEED TO DISCUSS HOW TactileInterface is really implemented
        self.tactile_sensor_block = TactileForce(camera_id, tactile_threshold)
    
    # setpoint, measure_variable, external_tracking_signal

    def forward(self, error):
        # We may need to discretize sensor reading values
        # If the defined max_output_val is too high and the
        # sensor never actually reaches that value the gripper 
        # will neever be able to close completely.
        # IN other words we need to experiment and see what is the 
        # maximum value we can actually get.

        # Min-max normalization
        offset = (65 - self.min_output_val)/(self.max_output_val - self.min_output_val) 
        normalized_error = (error - self.min_output_val)/(self.max_output_val - self.min_output_val) #+ offset 
        print(f"Normalized error: {normalized_error}")
        
        delta_x = self.pid_block.forward(normalized_error)
        #delta_x = 0.1 * delta_x # FOR SAFETY WHILE TESTING ARM, DELETE ONCE 
        print(f"PID output: {delta_x}")
        
        # The gripper block is receiving inputs between 0 and 1
        # 0 means completely opened, 1 means completely closed

        # Given that we are not measuring the gripper distance
        # there's nothing to return, on feedback we are going to 
        # measure the force

        #88888888888888888888888888888
        # READ GRIPPER POSITION
        #88888888888888888888888888888
        new_position = delta_x + self.gripper_block.gripper_position()
        new_position = 0 if new_position < 0 else 1 if new_position > 1 else new_position
        print(f"New position: {new_position}")
        self.gripper_block.gripper_width(new_position)
        

    def feedback(self, magnitude=False):
        """
        magnitude:  type bool
        """
        force_x, force_y = self.tactile_sensor_block.get_components(self.tactile_sensor_block.cap)
        
        self.force_x_filter.append(force_x)
        self.force_y_filter.append(force_y)
        
        if len(self.force_x_filter) > 10:
            self.force_x_filter.pop(0)
            self.force_y_filter.pop(0)

        avg_force_x = sum(self.force_x_filter)/len(self.force_x_filter)
        avg_force_y = sum(self.force_y_filter)/len(self.force_y_filter)
        
        if magnitude:
            avg_mag = math.sqrt(avg_force_x**2 + avg_force_y**2)
            return avg_mag
        else:
            return avg_force_x, avg_force_y
        
        

