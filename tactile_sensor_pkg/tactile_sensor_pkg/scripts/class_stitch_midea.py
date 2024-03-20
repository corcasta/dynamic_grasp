# %%
import numpy as np
import cv2
import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from skimage import io
from tactile_sensor_pkg.scripts import PKG_DIR

sensor_id = '04'

# %%
class Stitch(object):

    def get_coordinates(self,image):
        
        with open(PKG_DIR + "/test_img/config/sensor_" + sensor_id + ".json", "r") as f:
            data = json.load(f)
            ncol = data["ncol"]
            nrow = data["nrow"]  
            x_0 = data["x_0"]
            y_0 = data["y_0"]
            x_end = data["x_end"]
            y_end = data["y_end"]
            w = data["w"]
            h = data["h"]

        self.rect = []
        for i in range(ncol):
            row = []
            for j in range(nrow):
                center_x = int(x_0 + j * (x_end - x_0) / (nrow - 1) if nrow > 1 else x_0 + (x_end - x_0) / 2)
                center_y = int(y_0 + i * (y_end - y_0) / (ncol - 1) if ncol > 1 else y_0 + (y_end - y_0) / 2)
                x1 = int(center_x - w / 2)
                y1 = int(center_y - h / 2)
                x2 = int(center_x + w / 2)
                y2 = int(center_y + h / 2)
                row.append(((x1, y1), (x2, y2), (center_x, center_y)))
            self.rect.append(row)

        self.image = image

        k = 1
        left_top1 = [0,0]
        right_bottom1 = [0,0]


        for i in range(ncol):
            for j in range(nrow):
                (x1, y1), (x2, y2), _ = self.rect[i][j]  # 获取当前矩形的左下角、右上角坐标

                left_top = (x1, y1)
                right_bottom = (x2, y2)
                
                # Create a Rectangle patch
                rect_patch = patches.Rectangle((left_top[0],left_top[1]),
                                        right_bottom[0]-left_top[0],right_bottom[1] - left_top[1],
                                        linewidth=2, edgecolor='w',facecolor='none')
                # Add the patch to the Axes
                # ax.add_patch(rect_patch)

                # cut roi tiles and save
                img_tile = self.image[int(left_top[1]):int(right_bottom[1]), int(left_top[0]):int(right_bottom[0]), :]

                k = k + 1

    def original_pic(self):
        ncol = 3
        nrow = 3

        self.vstack = []
        tile_list = []
        for i in range(nrow):
            hstack = []
            for j in range(ncol):
                (x1, y1), (x2, y2), _ = self.rect[i][j]  # 获取当前矩形的左上角、右下角坐标

                left_top = (x1, y1)
                right_bottom = (x2, y2)

                tile = self.image[int(left_top[1]):int(right_bottom[1]),
                            int(left_top[0]):int(right_bottom[0]), :]
                tile_flipped = cv2.flip(tile, -1)
                
                tile_list.append(tile_flipped)
                if hstack == []:
                    hstack = tile_flipped
                else:
                    hstack = np.hstack((hstack, tile_flipped))
            if self.vstack == []:
                self.vstack = hstack
            else:
                self.vstack = np.vstack((self.vstack, hstack))
            
        self.output = self.vstack.copy()

        return self.vstack

    def divide(self):
        self.output_arr = []
        step = 230
        for i in range(3):
            for j in range(3):
                output = self.output[i * step:(i + 1) * step, j * step:(j + 1) * step]
                self.output_arr.append(output)
            
    def modify_hsv(self, image, hue_shift, saturation_scale, value_scale):

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Split the HSV image into individual channels
        h, s, v = cv2.split(hsv_image)

        # Modify the HSV channels
        h = (h + hue_shift) % 180
        s = np.clip(s * saturation_scale, 0, 255).astype('uint8')
        v = np.clip(v * value_scale, 0, 255).astype('uint8')

        # Merge the modified channels back into an HSV image
        modified_hsv = cv2.merge((h, s, v))

        # Convert the modified HSV image back to BGR color space
        modified_bgr = cv2.cvtColor(modified_hsv, cv2.COLOR_HSV2BGR)

        return modified_bgr
    
    # def crop(self,output,y0,x0,leny,lenx,hsv):
    #     o = output[y0:y0+leny,x0:x0+lenx]
    #     o = self.modify_hsv(o, hsv[0], hsv[1], hsv[2])

    #     return o

    def crop(self,output,up_left,down_right,hsv):
        y0 = up_left[0]
        y1 = down_right[0]
        x0 = up_left[1]
        x1 = down_right[1]
        o = output[y0:y1,x0:x1]
        o = self.modify_hsv(o,hsv[0], hsv[1], hsv[2])
        return o

    def stitch_row(self,o1,o2,o3):

        result = np.concatenate((o1,o2,o3),axis=1)

        return result

    def stitch_col(self,o1,o2,o3):

        result = np.concatenate((o1,o2,o3),axis=0)

        return result
    
    def draw_rectangle(self,output,y0,x0,leny,lenx):
        rec = cv2.rectangle(output,(x0,y0),(x0+lenx,y0+leny),color=(0,0,255),thickness=2)

        return rec

    def all_stitch(self):

        for i in range(1,10):
            with open(PKG_DIR + "/config/tactile_config/tactile_4/crop_{}.json".format(i), "r") as f:
                data = json.load(f)
                left_upper_x = data["left_upper_x"]
                left_upper_y = data["left_upper_y"]
                right_down_x = data["right_down_x"]
                right_down_y = data["right_down_y"]
            globals()[f'c{i}'] = self.crop(self.output_arr[i-1],[left_upper_y,left_upper_x],[right_down_y,right_down_x],[0,1,1])

        # print("c1",c1)    

        # leny_row1 = 2000

        # c1_ = self.crop(self.output1,[20,23],[61,116],[0,1,1])
        # c2 = self.crop(self.output2,[20,58],[61,173],[0,1,1])
        # c3 = self.crop(self.output3,[22,126],[63,220],[0,1,1])

        s1 = self.stitch_row(c1,c2,c3)
        # cv2.imshow('s1',s1)

        # leny_row2 = 150
        # c4 = self.crop(self.output4,[8,20],[138,113],[0,1,1])
        # c5 = self.crop(self.output5,[13,50],[143,165],[0,1,1])
        # c6 = self.crop(self.output6,[8,110],[138,204],[0,1,1])

        s2 = self.stitch_row(c4,c5,c6)

        # leny_row3 = 190
        # c7 = self.crop(self.output7,[62,27],[207,120],[0,1,1])
        # c8 = self.crop(self.output8,[80,62],[225,177],[0,1,1])
        # c9 = self.crop(self.output9,[75,136],[220,230],[0,1,1])

        s3 = self.stitch_row(c7,c8,c9)
        output = self.stitch_col(s1,s2,s3)

        return output
    
    def save_image(self,image,addr,num):
        address = addr + str(num)+ '.jpg'
        cv2.imwrite(address,image)
            
    def draw_black(self,image,param_file):

        loc_0 = []

        # param_file = PKG_DIR + "/test_img/pick_and_place_treatment/2/tactile/blob.json'
        # param_file = PKG_DIR + "/test_img/pick_and_place_treatment/4/draw_black.json'
        # param_file = PKG_DIR + "/test_img/config/draw_black_03.json'

        with open(param_file, "r") as json_file:
                    loaded_params = json.load(json_file)

        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = loaded_params["filterByColor"]
        params.blobColor = loaded_params["blobColor"]
        params.minThreshold = loaded_params["minThreshold"]
        params.maxThreshold = loaded_params["maxThreshold"]
        params.filterByArea = loaded_params["filterByArea"]
        params.minArea = loaded_params["minArea"]
        params.maxArea = loaded_params["maxArea"]
        params.filterByCircularity = loaded_params["filterByCircularity"]
        params.minCircularity = loaded_params["minCircularity"]
        params.filterByConvexity = loaded_params["filterByConvexity"]
        params.minConvexity = loaded_params["minConvexity"]
        params.filterByInertia = loaded_params["filterByInertia"]
        params.minInertiaRatio = loaded_params["minInertiaRatio"]

        detector = cv2.SimpleBlobDetector_create(params)

        clane = cv2.createCLAHE(clipLimit=2.5, tileGridSize = (9,9))

        gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)

        img_eq = clane.apply(gray)
        keypoints = detector.detect(img_eq)
        loc = []

        for i in range(0, len(keypoints)):
            loc.append([keypoints[i].pt[0], keypoints[i].pt[1]])
        loc = np.array(loc)

        for kp in keypoints:
            x,y = np.int32(kp.pt)
            radius = int(kp.size / 2)
            if radius <=7:
                
                radius = radius+5
            # print(radius)
            global blob
            cv2.circle(image,(x,y),radius,0,thickness = -1)

        return image,loc,keypoints
#%%
    def classify(self,hsv_image,lighter,color):
        # Create masks for conditions 1 and 2
        mask_lighter = hsv_image[:, :, 2] > lighter
        mask_color = hsv_image[:, :, 0] > color

        hsv_image[mask_lighter | mask_color, 0] = 0
        hsv_image[mask_lighter | mask_color, 1] = 0
        hsv_image[mask_lighter | mask_color, 2] = 255

        return hsv_image

    def classify_RGB(self, test, n1, n2, n3):
        cond1 = test[:,:,0] > n1
        cond2 = test[:,:,1] > n2
        cond3 = test[:,:,2] > n3

        test[cond1 | cond2 | cond3, 0] = 255
        test[cond1 | cond2 | cond3, 1] = 255
        test[cond1 | cond2 | cond3, 2] = 255

        return test
       
        
#%%
if __name__ == "__main__":
    stitch = Stitch()
    img_list = []

#%%
    #  stitch
    for i in range(1001):

        image = cv2.imread(PKG_DIR + "/test_img/pick_and_place_treatment/4/sensor16/sensor_{}.jpg".format(i))

        stitch.get_coordinates(image)
        original_pic = stitch.original_pic()
        stitch.divide()

        stitched_image,_ = stitch.all_stitch()

        stitched_image = stitched_image[:,20:-30]

        # stitch.save_image(stitched_image,PKG_DIR + "/test_img/pick_and_place_treatment/4/tactile/stitch_2/stitch_',int(i))

#%%    
    # draw black blob
    for i in range(1001):
        img = cv2.imread(PKG_DIR + "/test_img/pick_and_place_treatment/TACTILE6/stitch/stitch_{}.jpg".format(i))
        blob,_,_ = stitch.draw_black(img)
        # stitch.save_image(blob,PKG_DIR + "/test_img/pick_and_place_treatment/TACTILE6/blob/blob_",int(i))
    
#%%
    # draw boundary
    for i in range(1):

        image = cv2.imread(PKG_DIR + "/test_img/pick_and_place_treatment/2/sensor12/sensor_0.jpg")

        stitch.get_coordinates(image)
        original_pic = stitch.original_pic()
        stitch.divide()

        _,rectangle = stitch.all_stitch()

        # stitch.save_image(rectangle,PKG_DIR + "/test_img/pick_and_place_treatment/boundary/boundary",24)

    #%%
    # example
    image = cv2.imread(PKG_DIR + "/test_img/pick_and_place_treatment/FC1/2023-11-10_15-35-05/4_x.jpg")

    stitch.get_coordinates(image)
    original_pic = stitch.original_pic()
    stitch.divide()

    stitched_image,_ = stitch.all_stitch()

    stitched_image = stitched_image[:,20:-30]

#%%
    #hsv example
    for k in range(1000):

        image = cv2.imread(PKG_DIR + "/test_img/pick_and_place_treatment/TACTILE6/stitch/stitch_{}.jpg".format(k))
        image = image[:,20:-30]
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        fig = plt.figure(figsize=(7, 7))
        ax1 = fig.add_subplot(2, 4, 1)
        ax2 = fig.add_subplot(2, 4, 2)
        ax3 = fig.add_subplot(2, 4, 3)
        ax4 = fig.add_subplot(2, 4, 4)
        ax5 = fig.add_subplot(2, 4, 5)
        ax6 = fig.add_subplot(2, 4, 6)
        ax7 = fig.add_subplot(2, 4, 7)

        ax1.imshow(hsv_image), ax1.set_title('HSV')

        hsv_image = stitch.classify(hsv_image)

        ax2.imshow(hsv_image, cmap='gray'), ax2.set_title('HSV_2')
        
        rgb1 = cv2.cvtColor(hsv_image,cv2.COLOR_HSV2RGB)

        ax3.imshow(rgb1, cmap='gray'), ax3.set_title('RGB')

        # rgb2 = classify_RGB(rgb1,160,160,160) #160,130,130
        rgb2 = stitch.classify_RGB(rgb1,160,130,130)
        
        ax4.imshow(rgb2, cmap='gray'), ax4.set_title('RGB_2')

        # stitch.save_image(rgb1,PKG_DIR + "/test_img/pick_and_place_treatment/TACTILE6/hsv/hsv_',int(k))    
