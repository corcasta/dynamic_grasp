#%%
import cv2
import numpy as np
import json
from sklearn.neighbors import KDTree
from scipy.interpolate import griddata
from tactile_sensor_pkg.src.tactile.nHHD import *
from tactile_sensor_pkg.src import getStitched
#import ..src.getStitched as getStitched

import matplotlib.pyplot as plt
from matplotlib.colors import ColorConverter as cc
from numpy import flip
import math
from matplotlib.colors import LinearSegmentedColormap, Normalize
import matplotlib.colors as mcolors
from matplotlib.cm import get_cmap


def create_custom_cmap():

    hsv_values = [
        (240 / 360.0, 100 / 100.0, 100 / 100.0), # blue
        (120 / 360.0, 100 / 100.0, 100 / 100.0),  # Green
        (100 / 360.0, 90 / 100.0, 80 / 100.0),  # Yellow
        # (70 / 360.0, 100 / 100.0, 100 / 100.0),  # Yellow
        (0 / 360.0, 100 / 100.0, 100 / 100.0)   # Red
    ]

    # Convert HSV to RGB

    rgb_values = [mcolors.hsv_to_rgb(hsv) for hsv in hsv_values]# Create the colormap using LinearSegmentedColormap

    # Create the colormap using LinearSegmentedColormap
    cmap_name = "custom_green_to_yellow_to_red"
    custom_cmap = mcolors.LinearSegmentedColormap.from_list(cmap_name, rgb_values)

    return custom_cmap


class TACTILE(KDTree, nHHD):

    def __init__(self, vertice_file, param_file, 
                 grid_spacing = (30, 30), threshold = 45):
        
        self.grid_spacing = grid_spacing
        self.threshold = threshold

        self.loc_0 = []
        self.grid_flat = np.array([])
        self.loc = np.array([])
        self.recent_loc = np.array([])

        self.rect, _ = getStitched.get_vertice(vertice_file)
        # Assign the loaded parameters back to the SimpleBlobDetector_Params object
        with open(param_file, "r") as json_file:
            loaded_params = json.load(json_file)

        self.params = cv2.SimpleBlobDetector_Params()
        self.params.filterByColor = loaded_params["filterByColor"]
        self.params.blobColor = loaded_params["blobColor"]
        self.params.minThreshold = loaded_params["minThreshold"]
        self.params.maxThreshold = loaded_params["maxThreshold"]
        self.params.filterByArea = loaded_params["filterByArea"]
        self.params.minArea = loaded_params["minArea"]
        self.params.maxArea = loaded_params["maxArea"]
        self.params.filterByCircularity = loaded_params["filterByCircularity"]
        self.params.minCircularity = loaded_params["minCircularity"]
        self.params.filterByConvexity = loaded_params["filterByConvexity"]
        self.params.minConvexity = loaded_params["minConvexity"]
        self.params.filterByInertia = loaded_params["filterByInertia"]
        self.params.minInertiaRatio = loaded_params["minInertiaRatio"]
        self.set_params()


    def set_params(self):
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:
            self.detector = cv2.SimpleBlobDetector(self.params)
        else:
            self.detector = cv2.SimpleBlobDetector_create(self.params)

        self.clane = cv2.createCLAHE(clipLimit=2.5,tileGridSize=(9,9))


    def blob_detect(self,image):
        # self.tiles = getStitched.get_packed_tiles(self.img.copy(), self.rect)
        self.tiles = image
        self.gray = cv2.cvtColor(self.tiles.copy(), cv2.COLOR_BGR2GRAY)

        self.img_eq = self.clane.apply(self.gray)
        self.keypoints = self.detector.detect(self.img_eq)
        self.loc = []

        for i in range(0, len(self.keypoints)):
            self.loc.append([self.keypoints[i].pt[0], self.keypoints[i].pt[1]])
        self.radii = [kp.size / 2 for kp in self.keypoints]
        self.loc = np.array(self.loc)
        # print('loc1',self.loc)

        return self.loc, self.keypoints, self.radii
    

    def find_ref(self, img):
        self.img = img
        self.width, self.length, _ = self.img.shape
        # print(self.width, self.length)
        self.loc, self.keypoints,self.radii = self.blob_detect(img)
 
        # self.grid_x, self.grid_y = np.mgrid[0:self.length:32, 
        #                                    0:self.width-100:22] # use this interval to adjust the size of matrix (8,6)
        # self.grid_x, self.grid_y = np.mgrid[49:self.length-40:22, 
        #                             29:self.width-52:30]
        self.grid_x, self.grid_y = np.mgrid[10:self.length-10:35, 
                                    20:self.width-20:40]
        #---X Axis and Y Axis---
        self.x_axis = self.grid_x.reshape(-1).astype(np.int64)
        self.y_axis = self.grid_y.reshape(-1).astype(np.int64)

        if self.loc.shape[0] > 0:
            self.loc_0 = self.loc.copy()

            rm_index = []
               
            self.loc = np.delete(self.loc_0, rm_index, 0)
            self.loc_0 = self.loc.copy()
            self.recent_loc = self.loc.copy()

            return 1
        
        else:
            return 0
    
    def reset_ref(self, cap):    
        while True:
            ret, frame = cap.read()
            if self.find_ref(frame):
                break
    
    def points_to_area(self,points):
        area = 0
        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i+1]
            area += (x1 * y2) - (x2 * y1)

        area = abs(area) / 2
        return area

    def get_disp(self, img):
        self.img = img
        self.loc, self.keypoints,self.radii = self.blob_detect(img)
        

        if self.loc.shape[0] > 0 and self.recent_loc.shape[0] > 0:
            kdt = KDTree(self.loc, leaf_size=45, metric='euclidean')

            dist, ind = kdt.query(self.recent_loc, k=1)

            thd = (dist < self.threshold) * 1
            thd_nz = np.where(thd)[0]
            # update point if close enough point are detected
            self.recent_loc[thd_nz] = np.reshape(self.loc[ind[thd_nz]], (len(thd_nz), 2))

            scalar_list = [item[0] for item in ind[thd_nz]]


            self.radius_list = [self.radii[i] for i in scalar_list]

            loc_v = self.recent_loc - self.loc_0  # diff vector

            self.grid_vector = griddata(self.recent_loc, loc_v, 
                                        (self.grid_x, self.grid_y), method='cubic')
            self.grid_vector[np.isnan(self.grid_vector)] = 0            

        return self.grid_vector
        
    def draw_grid_vector(self,img,grid_vector):
        # print(grid_vector)
        rows, cols = 8,8
        x = np.linspace(30, img.shape[1], cols, endpoint=False)
        y = np.linspace(30, img.shape[0], rows, endpoint=False)
        grid_x, grid_y = np.meshgrid(x, y)

        grid_vector_x = grid_vector[:,:,0]
        grid_vector_y = grid_vector[:,:,1]


        color = (0, 0, 255)  # Red color
        line_thickness = 1
        for i in range(rows):
            for j in range(cols):
                
                start_point = (int(grid_x[i, j]), int(grid_y[i, j]))
                final_point = (int(grid_x[i, j])+int(grid_vector_x[j,i]), int(grid_y[i, j])+int(grid_vector_y[j,i]))
                cv2.arrowedLine(img, start_point, final_point, color, line_thickness)
        
        # for i in range(rows):
        #     for j in range(cols):
        #         dis_x = grid_vector[:,:,0]
        #         dis_y = grid_vector[:,:,1]
        #         center = (int(grid_x[i, j]), int(grid_y[i, j]))
        #         # cv2.circle(img, center, 1, (0,0,255), -1)
        return img




    def draw_2D(self,image,recent_loc):

        # the number of the blobs
        if self.loc.shape[0]>=5:
            recent_l = recent_loc
        else:
            recent_l = self.loc_0

        # print('re', recent_l)
        recent_loc_x = recent_l[:,:1].reshape(-1).astype(np.int64)
        recent_loc_y = recent_l[:,1:].reshape(-1).astype(np.int64)
        # print(recent_loc_x)
        loc_0_x = self.loc_0[:,:1].reshape(-1).astype(np.int64)
        loc_0_y = self.loc_0[:,1:].reshape(-1).astype(np.int64)
        # print(loc_0_x)
        ratio = 1
        for i in range(recent_l.shape[0]):

            dis_x = recent_loc_x[i]-loc_0_x[i]
            dis_y = recent_loc_y[i]-loc_0_y[i]
            cv2.arrowedLine(image,(loc_0_x[i],loc_0_y[i]),
                            (dis_x*ratio+loc_0_x[i],dis_y*ratio+loc_0_y[i]),
                            (0,0,255),line_type=cv2.LINE_AA, thickness = 2, tipLength=0.3)
        return image
    
    def draw_2D_color(self,image,recent_loc):

        if self.loc.shape[0]>=5:
            recent_l = recent_loc
        else:
            recent_l = self.loc_0

        recent_loc_x = recent_l[:,:1].reshape(-1).astype(np.int64)
        recent_loc_y = recent_l[:,1:].reshape(-1).astype(np.int64)

        loc_0_x = self.loc_0[:,:1].reshape(-1).astype(np.int64)
        loc_0_y = self.loc_0[:,1:].reshape(-1).astype(np.int64)

        ratio = 1

        for i in range(recent_l.shape[0]):

            dis_x = recent_loc_x[i]-loc_0_x[i]
            dis_y = recent_loc_y[i]-loc_0_y[i]
            # magnitude = np.sqrt(dis_x**2+dis_y**2)
            magnitude = abs(dis_y)
            norm = Normalize(0,20)
            norms = norm(magnitude)
            # print('norms', norms)
            # cmaps = create_custom_cmap()
            cmaps = get_cmap('turbo')
            if norms <= 0.1:
                norms = norms + 0.1
            
            colors = cmaps(norms)
            arrow_colors = colors
            # print("arrow_colors",arrow_colors)
            cv2.arrowedLine(image,(loc_0_x[i],loc_0_y[i]),
                            (dis_x*ratio+loc_0_x[i],dis_y*ratio+loc_0_y[i]),
                            (arrow_colors[2]*255,arrow_colors[1]*255,arrow_colors[0]*255),
                            line_type=cv2.LINE_AA, thickness = 2, tipLength=0.5)
        return image
    
