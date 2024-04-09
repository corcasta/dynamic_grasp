import os   
import sys
import argparse
import pandas as pd
from smart_gripper_pkg.scripts.utilities import DeviceConnection, parseConnectionArguments
from simple_pid import PID
from smart_gripper_pkg.scripts.gripper_interface2 import Gripper
from smart_gripper_pkg.scripts.gelsight_interface import Gelsigth
from smart_gripper_pkg.scripts.filter_interface import LiveLFilter, LiveSosFilter

import time
import scipy.signal
from scipy.signal import butter, filtfilt
from apscheduler.schedulers.background import BackgroundScheduler
import numpy as np


position = 0
slip_flag = True
slip_threshold = 0.5
mag_tick = 0
mag_tack = 0

def slip_detection():
    print("Callback slip detection")
    global slip_flag, mag_tick, mag_tack
    delta_mag = np.abs(mag_tack-mag_tick)
    slope = delta_mag/0.33

    if slope >= slip_threshold:
        slip_flag = True
    
    mag_tick = mag_tack


def update_position():
    global position
    position += 2
    #print("New position updated")


def main(): 
    global position, mag_tick, mag_tack, slip_flag
    # STEP-1) Creating connection to the gripper
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = parseConnectionArguments(parser)
    # Setup API
    device = DeviceConnection.createTcpConnection(args)
    router = device.login()
    device2 = DeviceConnection.createUdpConnection(args)
    router_real_time = device2.login()

    print("LOGGING IN to Gripper Session")
    force_sensor = Gelsigth()
    gripper = Gripper(router, router_real_time)
    
    
    # ********************************************
    window_gmv = []
    history_gmv = []
    history_fmv = []
    history_fmv2 = []
    history_e = []
    history_sp = []
    history_fx = []
    history_fy = []
    history_fz = []

    history_fx_filtered = []
    history_fy_filtered = []
    history_fz_filtered = []

    
    history_time = []
    time_accum = 0
    # ********************************************
    
    #sched = BackgroundScheduler()


    # Initial loop
    fz = 0
    
    position = 0
    #while fz < 5:
    #    gripper.Goto(position)
    #    _, _, fz = force_sensor.get_forces()
    #    #time.sleep(2)
    #    position += 0.01
    #    print("Init_position {:.3f} Fz: {:.3f}".format(position, fz))
    
    print("Finishing Initial Gripper position")


    try:
        gripper.Goto(position)
        

        fx, fy, fz = force_sensor.get_forces()
        window_forces = np.array([fx, fy, fz])
        
        mag_tick = np.sqrt(fx**2 + fy**2 + fz**2)
        mag_tack = mag_tick
        time.sleep(2)
        print("Finish waiting")
        iteration = 0
        #sched.start()
        #job = sched.add_job(slip_detection, "interval", seconds=0.33)
        

        while True:
            start_time = time.time()
            fx, fy, fz = force_sensor.get_forces()
            forces = np.array([fx, fy, fz])
           
            window_forces = np.vstack((window_forces, forces))
            if len(window_forces) > 20:
                window_forces = window_forces[1:,:]
            filtered_forces = window_forces.sum(axis=0)/len(window_forces)
            
            mag_tack = np.sqrt((filtered_forces*filtered_forces).sum())
            #gripper.Goto(position)
            end_time = time.time()
            time_accum += end_time - start_time 

            if iteration % 75 == 0:
                #print("H")
                slip_detection()

            if slip_flag:
                position += 1
                gripper.Goto(position)
                slip_flag = False


            history_fx.append(fx)
            history_fy.append(fy)
            history_fz.append(fz)
            history_fx_filtered.append(filtered_forces[0])
            history_fy_filtered.append(filtered_forces[1])
            history_fz_filtered.append(filtered_forces[2])
            history_time.append(time_accum)
            print("Time: {:.3f} \t Position: {} \t fx: {:.3f} \t fy: {:.3f} \t fz:{:.3f}".format(time_accum, position, fx, fy, fz))


    except KeyboardInterrupt:
        # STEP-3) Logout gripper session
        device.logout()
        device2.logout()
        #job.remove()   
        #scheduler.shutdown()

        csv_name = "slip_demo3"
        df = pd.DataFrame()
        df["time"] = history_time
        df["fx"] = history_fx
        df["fy"] = history_fy
        df["fz"] = history_fz
        df["fx_filtered"] = history_fx_filtered
        df["fy_filtered"] = history_fy_filtered
        df["fz_filtered"] = history_fz_filtered
        df.to_csv(f"{csv_name}.csv", index=False)
        print("LOGGING OUT from Gripper Session")
        print("BYE BYE :)")

if __name__ == "__main__":
    main()