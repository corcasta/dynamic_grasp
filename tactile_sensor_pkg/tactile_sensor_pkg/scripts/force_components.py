import cv2
import numpy as np
from .class_stitch_midea import Stitch
from .class_tactile_midea import TACTILE
from . import PKG_DIR

class ForceSensor(object):

    def __init__(self, camera_id, tacile_threshold=45):
        self.stitch = Stitch()
        self.touch = TACTILE(param_file=PKG_DIR + "/config/tactile_config/blob.json",
                             threshold=tacile_threshold)
        reference_path = "----/config/tactile_config/reference.png"
        self.refimage = cv2.imread(reference_path)
        
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


    def force_components(self, cap):
        try:
            _,original_image = cap.read()
            self.stitch.get_coordinates(original_image)
            original_pic = self.stitch.original_pic()
            self.stitch.divide()
            stitched_image = self.stitch.all_stitch()
            stitched_image = stitched_image[:,20:-30]
            blur_image = cv2.GaussianBlur(stitched_image, (5,5),0)
            hsv_image = cv2.cvtColor(blur_image,cv2.COLOR_BGR2HSV)
            hsv_image = self.stitch.classify(hsv_image,160,250) #160,250
            hsved_image = cv2.cvtColor(hsv_image,cv2.COLOR_HSV2RGB)
            blob_image,_,_ = self.stitch.draw_black(hsved_image,param_file = PKG_DIR + '/config/tactile_config/draw_black.json')    
            self.touch.find_ref(self.refimage)
            _,recent_loc,_,_ = self.touch.get_disp(blob_image)
        except:
            recent_loc = np.zeros(8,2)

        return recent_loc

