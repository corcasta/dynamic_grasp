import cv2
import numpy as np
import json
from matplotlib import cm


def intersection(rois):
    x1 = max(rois[0][0], rois[1][0])
    y1 = max(rois[0][1], rois[1][1])
    x2 = min(rois[0][0] + rois[0][2], rois[1][0] + rois[1][2])
    y2 = min(rois[0][1] + rois[0][3], rois[1][1] + rois[1][3])

    return (x1, y1, x2 - x1, y2 - y1)


def get_stereomap(file_name):
    with open(file_name, "r") as infile:
        data = json.load(infile)

    # 将 JSON 数据转换回 NumPy 数组和其他原始数据类型
    rectifyScale = data["rectifyScale"]
    img_size = np.array(data["img_size"])
    newCameraMatrixL = np.array(data["newCameraMatrixL"])
    distL = np.array(data["distL"])
    newCameraMatrixR = np.array(data["newCameraMatrixR"])
    distR = np.array(data["distR"])
    rot = np.array(data["rot"])
    trans = np.array(data["trans"])

    rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv2.stereoRectify(newCameraMatrixL, distL, 
                                                                               newCameraMatrixR, distR, 
                                                                               img_size, rot, trans, 
                                                                               alpha = rectifyScale)
    f = Q[2,3]
    baseline = abs(1/Q[3,2])
    c=np.array([-Q[0,3], -Q[1,3]])
    
    stereoMapL = cv2.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, img_size, cv2.CV_16SC2)
    stereoMapR = cv2.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, img_size, cv2.CV_16SC2)

    return stereoMapL, stereoMapR, f, baseline, c, Q


def disparity_to_depth(disparity, baseline, f):
    with np.errstate(divide='ignore', invalid='ignore'):
        depth_array = baseline*f/disparity
    return depth_array


def getDisparity(imgL, imgR, param):
    imgL_gray = cv2.cvtColor(imgL, cv2.COLOR_RGB2GRAY)
    imgR_gray = cv2.cvtColor(imgR, cv2.COLOR_RGB2GRAY)

    left_matcher = cv2.StereoSGBM_create(**param, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    left_disp = left_matcher.compute(imgL_gray, imgR_gray)
    right_disp = right_matcher.compute(imgR_gray, imgL_gray)

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    
    wls_filter.setLambda(8000)
    # sigmaColor典型范围值为0.8-2.0
    wls_filter.setSigmaColor(1.3)
    wls_filter.setLRCthresh(24)
    wls_filter.setDepthDiscontinuityRadius(3)
    filtered_disp = wls_filter.filter(left_disp, imgL_gray, disparity_map_right=right_disp)/16

    # filtered_disp[filtered_disp <= int(param['minDisparity'])] = np.nan
    filtered_disp[filtered_disp < 20] = np.nan

    return filtered_disp


def getDisparity_2(imgL_gray, imgR_gray, param, filter_param):
    
    left_matcher = cv2.StereoSGBM_create(**param, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    left_disp = left_matcher.compute(imgL_gray, imgR_gray)
    right_disp = right_matcher.compute(imgR_gray, imgL_gray)

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    
    wls_filter.setLambda(filter_param['Lambda'])
    wls_filter.setSigmaColor(filter_param['SigmaColor'])
    wls_filter.setLRCthresh(filter_param['LRCthresh'])
    wls_filter.setDepthDiscontinuityRadius(filter_param['DepthDiscontinuityRadius'])

    filtered_disp = wls_filter.filter(left_disp, imgL_gray, disparity_map_right=right_disp)/16
    filtered_disp[filtered_disp <= int(param['minDisparity'])] = np.nan
    # filtered_disp[filtered_disp < 1] = np.nan

    return filtered_disp


def create_disp_map(disparity):
    # Normalize the disparity array to the range 0-255 for visualization
    normalized_disparity = np.zeros_like(disparity, dtype=np.float32)
    normalized_disparity = cv2.normalize(disparity, normalized_disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    # Convert the normalized array to an 8-bit format
    disparity_map = np.uint8(normalized_disparity)

    return disparity_map


def create_depth_map(depth, cmap='jet'):
    """
    Converts single channel disparity map
    to an RGB depth map.
    Arguments:
      - disp: Quantized (uint8) disparity map with dimensions H x W 
      - cmap: a valid cmap named for use with matplotlib's 'get_cmap'
    
    Returns an RGB depth map with dimension H x W x 3.
    """
    normalized_depth_map = np.zeros_like(depth, dtype=np.float32)
    normalized_depth_map = cv2.normalize(depth, normalized_depth_map, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    # Convert the normalized array to an 8-bit format
    depth_map_8bit = np.uint8(normalized_depth_map)

    # Apply the 'jet' color map
    jet_color_map = cv2.applyColorMap(depth_map_8bit, cv2.COLORMAP_JET)

    return jet_color_map
