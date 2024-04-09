import os   
import sys
import argparse
import pandas as pd
from smart_gripper_pkg.scripts.utilities import DeviceConnection, parseConnectionArguments
from simple_pid import PID
from smart_gripper_pkg.scripts.gripper_interface import Gripper
from smart_gripper_pkg.scripts.gelsight_interface import Gelsigth
from smart_gripper_pkg.scripts.filter_interface import LiveLFilter, LiveSosFilter

import time
import scipy.signal
from scipy.signal import butter, filtfilt


def butter_lowpass_filter(data, cutoff, fs, order):
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

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
    
    # ********************************************
    Tu = 34.62-32.23 
    Ku = 0.00075    
    Kp = 0.00025
    Ki = 0.00000002*Ku/Tu
    Kd = 0.0000005#*Ku*Tu
    pid_params = {"Kp": 0.00051,#0.00075,#0.00075,
                  "Ki": 0.0,#0.0,#.0000005,#0.00009,#0.000001578,
                  "Kd": 0.0,#0.0,#.0000005,#0.00009,#0.000001578,
                  "setpoint": 5,
                  "output_limits": (-1, 1),
                  "proportional_on_measurement": False,
                  "differential_on_measurement": False}
    # ********************************************

    # ********************************************
    first_filter = True
    hb = 0.08 # Hysteresis band of 5% oof setpoint
    # ********************************************

    # ********************************************
    second_filter = False
    # define lowpass filter with 2.5 Hz cutoff frequency
    fs = 35  # sampling rate, Hz
    b, a = scipy.signal.iirfilter(4, Wn=2.5, fs=fs, btype="low", ftype="butter")
    live_lfilter = LiveLFilter(b, a)
    # ********************************************

    # ********************************************
    pid = PID(**pid_params)
    gripper = Gripper(router, sleep_time=0.001)
    force_sensor = Gelsigth()
    # ********************************************

    # ********************************************
    window_gmv = []
    history_gmv = []
    history_fmv = []
    history_fmv2 = []
    history_e = []
    history_sp = []
    history_time = []
    time_accum = 0
    # ********************************************

    delta_vel = 0
    vel = 0
    try:

        setpoint = pid_params["setpoint"] 
        #gripper.width(0.229)
        #print("After calling vel command")
        
        # ********** Testing new vel function ***********
        #print("Going Home")
        #gripper.vel(-0.1)
        #print("First vel executed")
        #gripper.vel(1)
        #print("Second vel executed")
        #gripper.vel(-0.1)
        #print("First vel executed")
        # ********** Testing new vel function ***********

        
        while True:
            start_time = time.time()
            _, _, measure_variable = force_sensor.get_forces()
            pid_input = measure_variable

            # Filtering Sensor Noise
            if first_filter:
                window_gmv.append(pid_input)
                if len(window_gmv) > 50:
                    window_gmv.pop(0)
                filter_measure_variable = sum(window_gmv)/len(window_gmv)
                pid_input = filter_measure_variable
                
            # Filtering Sensor Noise 2
            if second_filter:
                filter_measure_variable_2 = live_lfilter(pid_input)
                pid_input = filter_measure_variable_2


            error = setpoint - pid_input
            error_p = (error/setpoint)
            
            if error_p > hb:
                vel = pid(pid_input)*-1
                #vel += delta_vel 
                # This  first if statement is when pid output is to small
                # for the actual gripper to make it move
                #print(f"OG_Vel: {vel}")
                if abs(vel) < 0.005:
                    if vel < 0:
                        vel = -0.005
                    else:
                        vel = 0.005
                
                # This second if statement is just to make sure nothing 
                # goes out of bounds
                if vel > 1:
                    vel = 1
                elif vel < -1:
                    vel = -1
                gripper.vel(vel)

            elif error_p < -hb:
                vel = pid(pid_input)*-1
                #vel += delta_vel 
                # This  first if statement is when pid output is to small
                # for the actual gripper to make it move
                #print(f"OG_Vel: {vel}")
                if abs(vel) < 0.005:
                    if vel < 0:
                        vel = -0.005
                    else:
                        vel = 0.005
                
                # This second if statement is just to make sure nothing 
                # goes out of bounds
                if vel > 1:
                    vel = 1
                elif vel < -1:
                    vel = -1
                gripper.vel(vel)

            end_time = time.time()
            time_accum += end_time - start_time 
            print("Time: {:.3f} \t Setpoint: {:.3f} \t Fz: {:.3f} \t vel: {:.4f} \t DP: {:.4f} \t Error_P: {:.3f}".format(time_accum, setpoint, pid_input, vel, delta_vel, error_p)) 

            history_gmv.append(measure_variable)
            history_fmv.append(filter_measure_variable)
            #history_fmv2.append(filter_measure_variable_2)
            history_time.append(time_accum)
            history_e.append(error)
            history_sp.append(setpoint)

            """
            if 40 <= time_accum <= 45.0:
                setpoint = 12
                pid.setpoint = setpoint
            if 80 <= time_accum <= 85:
                setpoint = 17
                pid.setpoint = setpoint
            if 120 <= time_accum <= 125:
                setpoint = 20
                pid.setpoint = setpoint
            if 160 <= time_accum <= 165:
                setpoint = 27
                pid.setpoint = setpoint
            if 200 <= time_accum <= 205:
                setpoint = 15
                pid.setpoint = setpoint
            if 240 <= time_accum <= 245:
                setpoint = 5
                pid.setpoint = setpoint
            """
            
    except KeyboardInterrupt:
        # STEP-3) Logout gripper session
        csv_name = "zn_pid_tuned_vel_test1"
        device.logout()
        df = pd.DataFrame()
        df["time"] = history_time
        df["set_point"] = history_sp
        if first_filter:
            df["filter_measure_variable"] = history_fmv
        if second_filter:
            df["filter_measure_variable_2"] = history_fmv2
        df["gross_measure_variable"] = history_gmv
        df["error"] = history_e
        print("Time size: {} \t Filter size: {}".format(len(df["time"]), len(df["filter_measure_variable"])))
        df.to_csv(f"{csv_name}.csv", index=False)
        print("LOGGING OUT from Gripper Session")
        print("BYE BYE :)")

if __name__ == "__main__":
    main()