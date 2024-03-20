import numpy as np
from scipy import stats
from scipy.spatial.distance import pdist, squareform
from scipy.optimize import minimize
import time

try:
    import cupy as cp
except:
    print("CuPy is not availiable.")



def depth_to_vertices(depth, f, c):
    h, w = depth.shape
    x, y = np.meshgrid(np.arange(w), np.arange(h))
    x, y = x.astype(np.float32), y.astype(np.float32)
    
    z = depth
    x = (x - c[0]) * z / f
    y = (y - c[1]) * z / f
    
    vertices = np.stack((x, y, z), axis=-1)
    vertices = vertices.reshape(-1, 3)

    vertices = vertices[(~np.isnan(vertices[:, 2])) & ~np.isnan(vertices[:, 0]) & ~np.isnan(vertices[:, 1])
                        & ~np.isinf(vertices[:, 2]) & ~np.isinf(vertices[:, 0]) & ~np.isinf(vertices[:, 1])]

    return vertices


def remove_outliers(vertices_image):
    z = vertices_image[:, 2]
    lower, upper = np.percentile(z, [0.5, 99.5])
    mask = (z >= lower) & (z <= upper) & (z != 0)
    return vertices_image[mask]


"realsense depth accuracy"
def fit_plane(vertices_image):
    A = np.c_[vertices_image[:, 0], vertices_image[:, 1], np.ones(vertices_image.shape[0])]
    C, _, _, _ = np.linalg.lstsq(A, vertices_image[:, 2], rcond=None)
    normal = np.append(C[:2], -1)
    return normal / np.linalg.norm(normal)


def compute_tilts(fitted_plane_normal):
    target_normal = np.array([0, -1])
    horizontal_tilt = np.arccos(np.dot(fitted_plane_normal[1:], target_normal) / np.linalg.norm(fitted_plane_normal[1:]))
    vertical_tilt = np.arccos(np.dot(fitted_plane_normal[[0, 2]], target_normal) / np.linalg.norm(fitted_plane_normal[[0, 2]]))
    return horizontal_tilt, vertical_tilt


def rotate_vertices(vertices_image, horizontal_tilt, vertical_tilt):
    Ry = np.array([
        [np.cos(horizontal_tilt), 0, np.sin(horizontal_tilt)],
        [0, 1, 0],
        [-np.sin(horizontal_tilt), 0, np.cos(horizontal_tilt)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(vertical_tilt), -np.sin(vertical_tilt)],
        [0, np.sin(vertical_tilt), np.cos(vertical_tilt)]
    ])
    return vertices_image @ (Ry @ Rx).T


def evaluate_z_accuracy(vertices_images, ground_truths, N):
    medians_array = []
    for i in range(N):
        vertices_image = vertices_images[i]
        gt = ground_truths[i]

        # Compute the fitted plane and its normal
        fitted_plane_normal = fit_plane(vertices_image)

        # Compute tilts
        horizontal_tilt, vertical_tilt = compute_tilts(fitted_plane_normal)

        # Rotate vertices
        rotated_vertices = rotate_vertices(vertices_image, horizontal_tilt, vertical_tilt)

        # Compute depth error
        depth_rotated = rotated_vertices[:, 2]
        depth_error = abs(depth_rotated - gt)

        # Compute median value
        median_value = np.percentile(depth_error, 50)
        medians_array.append(median_value)

    # Compute average of medians array
    med_avr = np.mean(medians_array)

    # Compute and return Z-Accuracy signed
    # return 100 * med_avr / gt
    return med_avr


"realsense fill rate"

def compute_fill_rate(vertices_images, size, stereo_params):
    height, width = size
    offset=stereo_params["numDisparities"]+stereo_params["minDisparity"]
    total_count = (width - offset) * height
    valid_count = len(vertices_images)
    fill_rate = float(100 * valid_count / total_count)
    return fill_rate

def compute_average_fill_rate(depth_images):
    fill_rate_array = np.array([compute_fill_rate(depth_image) for depth_image in depth_images])
    fill_rate_avr = np.mean(fill_rate_array)
    return fill_rate_avr


"realsense temporal noise"

def compute_std_matrix(depth_images):
    depth_tensor = np.stack(depth_images, axis=-1)
    depth_tensor_no_zeros = np.where(depth_tensor != 0, depth_tensor, np.nan)
    std_matrix = np.nanstd(depth_tensor_no_zeros, axis=-1)
    return std_matrix

def compute_pixel_temporal_noise(depth_images):
    std_matrix = compute_std_matrix(depth_images)
    median_std = np.nanpercentile(std_matrix, 50)
    avg_z = np.nanmean(np.where(np.isnan(depth_images), 0, depth_images))
    return 100 * median_std / avg_z


"realsense rmse cuda"

def distance_to_plane_cuda(points, normal, point_on_plane):
    return cp.dot(points - point_on_plane, normal)

def fit_plane_rmse_cuda(points):
    centroid = points.mean(axis=0)
    u, s, vh = cp.linalg.svd(points - centroid)
    normal = vh[-1]
    return normal, centroid

def root_mean_square_error_cuda(errors):
    return cp.sqrt(cp.mean(errors ** 2))

def compute_rmse_cuda(vertices_image):
    # Convert the input to CuPy array if it's a NumPy array
    if not isinstance(vertices_image, cp.ndarray):
        vertices_image = cp.array(vertices_image)

    # Compute Fitted_Plane to Vertices_Image
    normal, centroid = fit_plane_rmse_cuda(vertices_image)

    # Compute Z_Error = Original_Z – Fitted_Z (using broadcasting)
    z_error = distance_to_plane_cuda(vertices_image, normal, centroid)

    # Compute RMSE = RootMeanSquare of Z_Error
    rmse = root_mean_square_error_cuda(z_error)

    # Output: 100*RMSE/Z
    return float(100 * rmse / cp.mean(vertices_image[:, 2]))


"optimized"

def fit_plane_rmse_optimized(points):
    points_cp = cp.asarray(points)
    centroid = points_cp.mean(axis=0)
    u, s, vh = cp.linalg.svd(points_cp - centroid)
    normal = cp.asnumpy(vh[-1])
    return cp.asnumpy(normal), cp.asnumpy(centroid)


def root_mean_square_error_optimized(errors):
    return np.sqrt(np.mean(errors ** 2))


def compute_rmse_optimized(vertices_image):
    
    # device = cp.cuda.Device(1)
    # with device:
    # Compute Fitted_Plane to Vertices_Image
    normal, centroid = fit_plane_rmse_optimized(vertices_image)
    # Compute Z_Error = Original_Z – Fitted_Z using broadcasting
    z_error = np.dot(normal, (vertices_image - centroid).T)

    # Compute RMSE = RootMeanSquare of Z_Error
    rmse = root_mean_square_error_optimized(z_error)

    # Output: 100*RMSE/Z
    return 100 * rmse / np.mean(vertices_image[:, 2])