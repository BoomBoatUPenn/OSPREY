import cv2
import numpy as np
from copy import deepcopy

from utils import *
from AR_tags import *


ROOT = "C:\\Schoolwork\\2022 Spring Semester\\Senior Design\\OSPREY\\perception\\gopro\\testing\\raw\\02.28.22\\videos\\"

# distance threshold for classifying occupied spaces
d_thresh = 2 # currently in px, but maybe should be in meters/frame of the world

# for testing on videos, make sure to set the AR tag size in AR_PlaneDetection() to 0.1462

def create_AR_world(frame, april, undistorter, resize_factor, dims):
    april_im = frame
    resized_shape = (int(dims[0]*resize_factor), int(dims[1]*resize_factor))
    april_im = cv2.resize(april_im, dsize=resized_shape)
    april_im, origins, ground_plane = april.detect_tags(april_im, set_K=True)
    tag_data = deepcopy(origins)
    distance, theta = april.computeState(origins)
    undistorted = undistorter.undistort(frame)
    april_im = deepcopy(undistorted)
    masked_im = colorThreshold(undistorted, "orange")
    det_map = fill_map(masked_im, deepcopy(ground_plane))
    draw_occupancy_map(ground_plane, det_map)

    if "boat" in origins.keys():
        for i, pt in enumerate(origins["boat"][0][1:]): # boat coordinate system
            axis = AXES[str(i)]
            boat_c = origins["boat"][0][0]
            cv2.circle(april_im, boat_c, 3, (255, 0, 122), -1)
            if axis == 'x':
                cv2.arrowedLine(april_im, boat_c, pt, (0, 0, 255))
                cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(0, 0, 255))
            elif axis == 'y':
                cv2.arrowedLine(april_im, boat_c, pt, (0, 255, 0))
                cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(0, 255, 0))
            elif axis == 'z':
                cv2.arrowedLine(april_im, boat_c, pt, (255, 0, 0))
                cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(255, 0, 0))
    if "ground" in origins.keys():
        for i, pt in enumerate(origins["ground"][0][1:]): # ground coordinate system overlayed onto boat center
            axis = AXES[str(i)]
            ground_c = origins["ground"][0][0]
            if axis == 'x':
                cv2.arrowedLine(april_im, ground_c, pt, (0, 0, 255))
                april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(0, 0, 255))
            elif axis == 'y':
                cv2.arrowedLine(april_im, ground_c, pt, (0, 255, 0))
                april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(0, 255, 0))
            elif axis == 'z':
                cv2.arrowedLine(april_im, ground_c, pt, (255, 0, 0))
                april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(255, 0, 0))
        if ground_plane is not None:
            ground_c = origins["ground"][0][0]
            for i, row in enumerate(ground_plane):
                for j, pt in enumerate(row):
                    if (i, j) not in det_map["occupied"].keys() and (i, j) in det_map["free"]:
                        cv2.circle(april_im, pt, 3, (0, 0, 255), -1)
                    else:
                        cv2.circle(april_im, pt, 3, (0, 0, 0), -1)
            cv2.circle(april_im, ground_c, 3, (255, 0, 122), -1)
    return april_im, masked_im, undistorted;

def fill_map(masked_im, ground_plane):
    occupied = {}
    free = set()
    det_map = {"occupied": None,
                "free": None}
    if ground_plane is not None:
        n, m = len(ground_plane), len(ground_plane[0])
        dims = masked_im[:2][::-1]
        for i in range(n):
            for j in range(m // 2):
                x, y = ground_plane[i][j]
                if x > 0 and y > 0:
                    window = masked_im[y-d_thresh:y+d_thresh+1, x-d_thresh:x+d_thresh+1, :]
                    truth_mask = window > 0
                    if np.any(truth_mask):
                        # compute density
                        occupied[(i, j)] = np.sum(truth_mask) / (3*((2*d_thresh)**2))
                    else:
                        free.add((i, j))
        det_map["occupied"] = occupied
        det_map["free"] = free
    return det_map;

def draw_occupancy_map(ground_plane, det_map):
    n, m = len(ground_plane), len(ground_plane[0])
    grid = np.ones((n,m,3), dtype=np.float32)
    for i in range(n):
        for j in range(m):
            if (i,j) in det_map["occupied"].keys() and (i,j) not in det_map["free"]:
                density = det_map["occupied"][(i,j)]
                if density > 0.20:
                    grid[i,j,:] = np.array([1.0 - density, 0.0, density])
    scale_factor = 2.5
    new_dims = (int(scale_factor*m), int(scale_factor*n))
    grid = cv2.resize(grid, new_dims)
    cv2.imshow("Occupancy", grid)
    return;


if __name__ == "__main__":

    cap = cv2.VideoCapture(ROOT + "GX010057.MP4")
    first = True
    while cap.isOpened():
        nmat, frame = cap.read()
        if nmat:
            if first:
                april = AR_PlaneDetection()
                undistorter = UndistortionModule()
                dims = frame.shape[:2][::-1]
                resize_factor = 1./4.
                cv2.namedWindow("Masked")
                cv2.namedWindow("April")
                cv2.namedWindow("Occupancy")
                cv2.namedWindow("Source")
                AXES = {'0': 'x',
                        '1': 'y',
                        '2': 'z'}
                first = False
            april_im, masked_im, undistorted_im = create_AR_world(frame, april, undistorter, resize_factor, dims)
            key = cv2.waitKey(1)
            if key != ord('q'):
                cv2.imshow("Masked", masked_im)
                cv2.imshow("April", april_im)
                cv2.imshow("Source", undistorted_im)
                key = cv2.waitKey(1)
            else:
                break;


    