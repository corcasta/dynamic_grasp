import os   
import sys
import argparse
import pandas as pd
from smart_gripper_pkg.scripts.utilities import DeviceConnection, parseConnectionArguments
from simple_pid import PID
from smart_gripper_pkg.scripts.gripper_interface import Gripper
from smart_gripper_pkg.scripts.gelsight_interface import Gelsigth

def main(): 
    # STEP-1) Creating connection to the gripper
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = parseConnectionArguments(parser)
    # Setup API
    device = DeviceConnection.createTcpConnection(args)
    router = device.login()
    print("LOGGING IN to Gripper Session")
    
    pid_params = {"Kp": 0.00085,#0.00075,
                  "Ki": 0.0000005,#0.00009,#0.000001578,
                  "Kd": 0.0000005,#0.00009,#0.000001578,
                  "setpoint": 5,
                  "output_limits": (-1, 1)}

    pid = PID(**pid_params)
    gripper = Gripper(router, sleep_time=0.001)
    force_sensor = Gelsigth()


    window_gmv = []
    history_gmv = []
    history_fmv = []
    history_e = []
    history_sp = []

    
    try:
        while True:
            _, _, gross_measure_variable = force_sensor.get_forces()
            error = pid_params["setpoint"] - gross_measure_variable
            control_variable = pid(gross_measure_variable)*-1
            control_variable = control_variable   
            #control_variable = control_variable#*0.1
            #new_position = control_variable + gripper.position()
            gripper.vel(control_variable)

            print("Fz: {:.3f} \t Control Value: {:.4f} \t Error: {:.3f}".format(gross_measure_variable, control_variable, error)) 

            ## Filtering Sensor Noise
            #window_gmv.append(gross_measure_variable)
            #if len(window_gmv) > 20:
            #    window_gmv.pop(0)
            #filter_measure_variable = sum(window_gmv)/len(window_gmv)

            #history_gmv.append(gross_measure_variable)
            #history_fmv.append(filter_measure_variable)
            #error = set_point - filter_measure_variable
            #history_e.append(error)
            #history_sp.append(set_point)
            #print(f"Target Force: {set_point} \t Measure Force: {filter_measure_variable} \t Error: {error}")
    
    except KeyboardInterrupt:
        # STEP-3) Logout gripper session
        device.logout()
        df = pd.DataFrame()
        df["set_point"] = history_sp
        df["filter_measure_variable"] = history_fmv
        df["gross_measure_variable"] = history_gmv
        df["error"] = history_e
        df.to_csv("control_demo_8.csv")
        print("LOGGING OUT from Gripper Session")
        print("BYE BYE :)")

if __name__ == "__main__":
    main()