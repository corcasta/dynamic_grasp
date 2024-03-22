import matplotlib.pyplot as plt
from tactile_sensor_pkg.scripts.class_tactile_midea import TACTILE
from tactile_sensor_pkg.scripts.class_stitch_midea import Stitch
import cv2
import numpy as np
import os
import pandas as pd
import time
import pandas as pd
from tactile_sensor_pkg.scripts import PKG_DIR

touch = TACTILE(vertice_file= PKG_DIR+"/config/tactile_vertice.json",
                    param_file=PKG_DIR+'/test_img/config/draw_black_04.json',
                    grid_spacing=(30, 30), threshold=30)
stitch = Stitch()


TACTILE_SENSOR_DEVICE_ID = 6
reference_path = PKG_DIR+"/config/tactile_config/tactile_4/reference3.png"
refimage = cv2.imread(reference_path)

if __name__=='__main__':

    cap = cv2.VideoCapture(TACTILE_SENSOR_DEVICE_ID)
    cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)



    # plt.ion()
    fig, ax1 = plt.subplots()
    line, = ax1.plot([],[],color='blue')
    ax2 = ax1.twinx()
    line_2, = ax2.plot([],[],color='red')

    # Set plot properties
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Tactile')
    ax1.set_ylim(0,320)
    ax2.set_ylabel('Stereo')
    ax2.set_ylim(0,70)
    ax1.set_title('Real-time Plot')
    ax1.grid(True)
    # Initialize variables
    x_data = []
    y_data1 = []
    y_data2 = []
    counter = 0
    history = []
    try:
        while True:
            _,original_image = cap.read()
            stitch.get_coordinates(original_image)
            original_pic = stitch.original_pic()
            stitch.divide()
            stitched_image = stitch.all_stitch()

            hsv_image = cv2.cvtColor(stitched_image,cv2.COLOR_BGR2HSV)
            hsv_image = stitch.classify(hsv_image,160,250) #160,250
            hsved_image = cv2.cvtColor(hsv_image,cv2.COLOR_HSV2RGB)
            blob_image,_,_ = stitch.draw_black(hsved_image,param_file=PKG_DIR+'/test_img/config/draw_black_04.json')
            touch.find_ref(refimage)
            flow_2d = touch.get_disp(blob_image)

            u = flow_2d[:,:,0].sum()
            v = flow_2d[:,:,1].sum()

            magnitude = np.sqrt(u**2+v**2)
            magnitude = np.sum(magnitude)
            m = np.sum(magnitude)
            print(m)

            ###########################################################################

            
            
            ###########################################################################

            # Add the new data to the lists
            x_data.append(counter)  # Use timestamps as x-axis value
            y_data1.append(m)
            y_data2.append(0)
            history.append(m)        
            if len(x_data) > 30:
                x_data.pop(0)
                y_data1.pop(0)
                y_data2.pop(0)

            # Update the plots
            # ax1.plot(x_data, y_data1, color='blue', label='Data 1')
            line.set_data(x_data, y_data1)
            ax1.set_xlim(counter-20,counter-1)

            line_2.set_data(x_data,y_data2)

            fig.canvas.draw()
            plt.pause(0.05)

            counter += 1
    except KeyboardInterrupt:
        df = pd.DataFrame(history)
        df.to_csv("standard_behaviour_2.csv")
        
