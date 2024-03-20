import cv2
import numpy as np
import json


def get_vertice(file_name):

    with open(file_name, "r") as f:
        data = json.load(f)
        ncol = data["ncol"]
        nrow = data["nrow"]
        x_0 = data["x_0"]
        y_0 = data["y_0"]
        x_end = data["x_end"]
        y_end = data["y_end"]
        w = data["w"]
        h = data["h"]

    size=(w,h)
    rect = []
    for i in range(ncol):
        row = []
        for j in range(nrow):
            center_x = int(x_0 + j * (x_end - x_0) / (nrow - 1) if nrow > 1 else x_0 + (x_end - x_0) / 2)
            center_y = int(y_0 + i * (y_end - y_0) / (ncol - 1) if ncol > 1 else y_0 + (y_end - y_0) / 2)
            x1 = int(center_x - w / 2)
            y1 = int(center_y - h / 2)
            x2 = int(center_x + w / 2)
            y2 = int(center_y + h / 2)
            # row.append(((x1, y1), (x2, y2), (center_x, center_y)))
            row.append(((x1, y1), (x2, y2)))
        rect.append(row)
        
    return rect, size


def get_vertice_stereo(file_name):

    with open(file_name, "r") as f:
        data = json.load(f)
        x = [data["x_0"], data["x_end"]]
        y = [data["y_0"], data["y_end"]]
        w = data["w"]
        h = data["h"]
    size=(w,h)
    rect = []
    for i in range(2):
        row = []
        x1 = int(x[i] - w / 2)
        y1 = int(y[i] - h / 2)
        x2 = int(x[i] + w / 2)
        y2 = int(y[i] + h / 2)
        row.append(((x1, y1), (x2, y2)))

        rect.append(row)
        
    return rect, size



def get_tiles(image, rect):
    tile_list = []

    ncol=np.array(rect).shape[0]
    nrow=np.array(rect).shape[1]
    
    for i in range(ncol):
        for j in range(nrow):
            (x1, y1), (x2, y2) = rect[i][j]  # 获取当前矩形的左下角、右上角坐标

            left_top = (x1, y1)
            right_bottom = (x2, y2)

            tile = image[int(left_top[1]):int(right_bottom[1]),
                        int(left_top[0]):int(right_bottom[0]), :]
            
            tile_list.append(tile)

    return tile_list


def get_undistorted_old(tile_list, stereoMapL, stereoMapR):
    imgL = cv2.rotate(np.ascontiguousarray(tile_list[0]),cv2.ROTATE_90_COUNTERCLOCKWISE)
    imgR = cv2.rotate(np.ascontiguousarray(tile_list[1]),cv2.ROTATE_90_COUNTERCLOCKWISE)

    undistortedR = cv2.remap(imgL, stereoMapL[0], stereoMapL[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    undistortedL = cv2.remap(imgR, stereoMapR[0], stereoMapR[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    return undistortedL, undistortedR


def get_undistorted(tile_list, stereoMapL, stereoMapR):

    imgL = np.transpose(tile_list[1], (1, 0, 2))
    imgR = np.transpose(tile_list[0], (1, 0, 2))

    undistortedL = cv2.remap(imgL, stereoMapL[0], stereoMapL[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    undistortedR = cv2.remap(imgR, stereoMapR[0], stereoMapR[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    return undistortedL, undistortedR


def get_packed_tiles(image, rect):
    vstack = []
    tile_list = []

    # ncol=np.array(rect).shape[0]
    # nrow=np.array(rect).shape[1]
    
    for i in range(3):
        hstack=[]
        for j in range(3):
            (x1, y1), (x2, y2) = rect[i][j]  # 获取当前矩形的左下角、右上角坐标

            left_top = (x1, y1)
            right_bottom = (x2, y2)

            tile = image[int(left_top[1]):int(right_bottom[1]),
                        int(left_top[0]):int(right_bottom[0]), :]
            
            tile_list.append(tile)
            if hstack == []:
                hstack = tile
            else:
                hstack = np.hstack((hstack, tile))
        if vstack == []:
            vstack = hstack
        else:
            vstack = np.vstack((vstack, hstack))

    return vstack


def find_ov(img1, img2, ov_pos, dim=1):
    if dim == 1:
        ov_1 = img1[:, -ov_pos:, :]
        ov_2 = img2[:,  :ov_pos, :]
    elif dim == 0:
        ov_1 = img1[-ov_pos:, :, :]
        ov_2 = img2[:ov_pos,  :, :]
    
    return ov_pos, ov_1, ov_2


def stitch(img1, img2, ov_pos, ov1, ov2, blend=True, dim=1): 
    
#     t1 = time.time()
    if blend:
        # linear blending
        ov1_blended = ov1.copy()
        ov2_blended = ov2.copy()
        for i in range(ov_pos):
            blend_coef = -(1/(ov_pos-1))*i + 1
            if dim==1:
                ov1_blended[:,i,:] = ov1[:,i,:]*blend_coef
                ov2_blended[:,i,:] = ov2[:,i,:]*(1.-blend_coef)
            elif dim==0:
                ov1_blended[i, :,:] = ov1[i, :,:]*blend_coef
                ov2_blended[i, :,:] = ov2[i, :,:]*(1.-blend_coef)
        ov_blended = ov1_blended + ov2_blended
        ov = ov_blended
    else:
        ov = ov1
#     print('time: linear blending >>>', time.time() - t1)
    
    if dim == 1:
        stitched_img = np.concatenate((img1[:, :-ov_pos, :], ov, img2[:, ov_pos:, :]), axis = 1)
    elif dim == 0:
        stitched_img = np.concatenate((img1[:-ov_pos, :, :], ov, img2[ov_pos:, :, :]), axis = 0)
    
    return stitched_img


def get_stitched_img(tile_list,stitch_calibration, nrow, ncol):
    img_stitched_height = None
    
    for i in range(ncol):
        img_stitched_width = None
        for j in range(nrow):
            if j == 0:
                continue
            else:
                img_1 =  tile_list[i*ncol + j-1]#tile_list_histMatched[i*4 + j-1]
                img_2 =  tile_list[i*ncol + j] #tile_list_histMatched[i*4 + j]
               
                ov_pos = int(stitch_calibration[i*ncol + j-1][1])
    #                 print(ov_pos)
                ov_pos, ov1, ov2 = find_ov(img_1, img_2, ov_pos, dim=1)
    #                 print('{}th overlap size in row {}: '.format(j-1, i), ov_pos)
                
                # stitch images with stacking
                img_stitched_width = stitch(img_stitched_width, img_2, ov_pos, ov1, ov2, dim=1)
                
#                 cv2.imshow("sw",img_stitched_width)
#                 cv2.waitKey()
#                 cv2.destroyAllWindows()

#             print(i,j)
#         print('Stitched row  width >>>  ', img_stitched_width.shape[1])

        if i == 0:
            img_stitched_height = img_stitched_width
        else:
            width = img_stitched_height.shape[1]
            width_new = img_stitched_width.shape[1]
            wdiff = width - width_new
    #             print(i, wdiff)
            if wdiff > 0:
                img_stitched_width = np.hstack((img_stitched_width, img_stitched_width[:, -wdiff:]))
            elif wdiff < 0:
                img_stitched_width = img_stitched_width[:, :wdiff]

            ov_pos_h = int(stitch_calibration[(i+1)*ncol - 1][1])
    #             print(ov_pos_h)
            ov_pos_h, ov_h1, ov_h2 = find_ov(img_stitched_height, img_stitched_width, ov_pos_h, dim=0)

    #             print('Row overlapping:', ov_pos_h)
            img_stitched_height = stitch(img_stitched_height, img_stitched_width, 
                                            ov_pos_h, ov_h1, ov_h2, dim=0)
    
    return img_stitched_height