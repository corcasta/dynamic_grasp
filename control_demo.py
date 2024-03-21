import os   
import sys
import argparse
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
    set_point = 120  # Force Magnitude we want to achieve
    controller_params = {
        "router": router,
        "camera_id": 7,
        "tactile_threshold": 45,
        "sleep_time": 0.1,
        "friction_coeff": 0.1,
        "max_output_val": 200.0,
        "min_output_val": 0.0,
        "gains": {"kp": 0.002, "ki": 0, "kd": 0.5}
        }
    
    force_controller = ForceController(**controller_params)
    measure_variable = force_controller.feedback(magnitude=True)
    error = set_point - measure_variable
    print(f"Target Force: {set_point} \t Measure Force: {measure_variable} \t Error: {error}")

    try:
        while True:
            force_controller.forward(error)
            measure_variable = force_controller.feedback(magnitude=True)
            error = set_point - measure_variable
            print(f"Target Force: {set_point} \t Measure Force: {measure_variable} \t Error: {error}")
    
    except KeyboardInterrupt:
        # STEP-3) Logout gripper session
        device.logout()
        print("LOGGING OUT from Gripper Session")
        print("BYE BYE :)")

if __name__ == "__main__":
    main()