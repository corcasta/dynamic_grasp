import os   
import sys
import argparse
import pandas as pd
from smart_gripper_pkg.scripts.utilities import DeviceConnection, parseConnectionArguments
from smart_gripper_pkg.scripts.dynamic_graspper import ForceController

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

    # STEP-2) Run closed loop system
    set_point = 0  # Force Magnitude we want to achieve
    controller_params = {
        "router": router,
        "sleep_time": 0.0,
        "friction_coeff": 0.1,
        "max_output_val": 100.0,
        "min_output_val": 0.0,
        "gains": {"Kp": 0.001, "Ki": 0, "Kd": 0, "setpoint":set_point}
        }
    
    window_gmv = []
    history_gmv = []
    history_fmv = []
    history_e = []
    history_sp = []

    force_controller = ForceController(**controller_params)
    gross_measure_variable = force_controller.feedback()
    
    # Filtering Sensor Noise
    window_gmv.append(gross_measure_variable)
    filter_measure_variable = sum(window_gmv)/len(window_gmv)

    error = set_point - filter_measure_variable
    print(f"Target Force: {set_point} \t Measure Force: {filter_measure_variable} \t Error: {error}")

    try:
        while True:
            force_controller.forward(error)
            gross_measure_variable = force_controller.feedback()
           
           # Filtering Sensor Noise
            window_gmv.append(gross_measure_variable)
            if len(window_gmv) > 20:
                window_gmv.pop(0)
            filter_measure_variable = sum(window_gmv)/len(window_gmv)

            history_gmv.append(gross_measure_variable)
            history_fmv.append(filter_measure_variable)
            error = set_point - filter_measure_variable
            history_e.append(error)
            history_sp.append(set_point)
            print(f"Target Force: {set_point} \t Measure Force: {filter_measure_variable} \t Error: {error}")
    
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