import cv2
import numpy as np
from time import time, sleep
import socket
from goprocam import GoProCamera
from goprocam import constants
from math import cos, pi
import random as rng
import json
import threading

from utils import *
from AR_tags import *
import keep_alive


# AXES:
AXES = {'0': 'x',
        '1': 'y',
        '2': 'z'}

def accept_json():
    """
    Accepts json configuration file.
    """
    with open('configs\\test_config.json', 'r') as f:
        PARAMS = json.load(f)
    return PARAMS;

def displayResults(imgs):
    """
    Displays processed images.
    inputs:
        - imgs: dictionary of images to be displayed
        - PARAMS: dictionary of parameters for display
    outputs:
        - (None)
    """
    if len(imgs) > 0:
        for i, (window, img) in enumerate(imgs.items()):
            if img is not None:
                cv2.imshow(window, img)

def main(params):
    """
    Main runner for stream-based image processing.  See utils for understanding function calls and
    the configs folder for understanding parameters.
    """
    imgs = {}
    for key in params.keys():
        if params[key] == True:
            imgs[key] = None
    
    for i, (window, value) in enumerate(params.items()):
        if value == True:
            if window == "april":
                april = AR_PlaneDetection()
            elif window == "undistort":
                undistorter = UndistortionModule()
            if window != "undistort":
                cv2.namedWindow(window)

    cap = cv2.VideoCapture("udp://127.0.0.1:10000")
    counter = 0
    t = time()
    while cap.isOpened():
        nmat, frame = cap.read()
        if nmat:
            if counter <= 3: # reduces effective frame rate
                counter += 1
                key = cv2.waitKey(1)
            else:
                if params["undistort"]: # camera lens undistortion
                    undistorted_im = frame
                    undistorted_im = undistorter.undistort(undistorted_im)
                    #imgs["undistort"] = undistorted_im
                
                if params["april"]: # april tag detection
                    april_im = frame
                    april_im, origins, ground_plane = april.detect_tags(april_im)
                    ground_origin_overlay, theta = april.calculateHeading(origins)
                    if params["undistort"]:
                        april_im = undistorted_im
                    if "boat" in origins.keys():
                        for i, pt in enumerate(origins["boat"][0][1:]): # boat coordinate system
                            axis = AXES[str(i)]
                            if axis == 'x':
                                cv2.arrowedLine(april_im, origins["boat"][0][0], pt, (0, 0, 255))
                                april_im = cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(0, 0, 255))
                            elif axis == 'y':
                                cv2.arrowedLine(april_im, origins["boat"][0][0], pt, (0, 255, 0))
                                april_im = cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(0, 255, 0))
                            elif axis == 'z':
                                cv2.arrowedLine(april_im, origins["boat"][0][0], pt, (255, 0, 0))
                                april_im = cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(255, 0, 0))

                        if ground_origin_overlay is not None:
                            for i, pt in enumerate(ground_origin_overlay[1:]): # ground coordinate system overlayed onto boat center
                                axis = AXES[str(i)]
                                if axis == 'x':
                                    cv2.arrowedLine(april_im, ground_origin_overlay[0], pt, (0, 0, 255))
                                    april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(0, 0, 255))
                                elif axis == 'y':
                                    cv2.arrowedLine(april_im, ground_origin_overlay[0], pt, (0, 255, 0))
                                    april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(0, 255, 0))
                                elif axis == 'z':
                                    cv2.arrowedLine(april_im, ground_origin_overlay[0], pt, (255, 0, 0))
                                    april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(255, 0, 0))
                    if ground_plane is not None:
                        for pt in ground_plane:
                            cv2.circle(april_im, pt, 3, (0, 0, 255), -1)
                    imgs["april"] = april_im
                
                if params["edges"] and not params["contours"]: # edge detection
                    if params["undistort"]:
                        edge_im = edgedetection(undistorted_im, params["contour_thresh"])
                    else:
                        edge_im = frame
                        edge_im = edgedetection(edge_im, params["contour_thresh"])
                    imgs["edges"] = edge_im

                if params["contours"]: # contour detection
                    if params["undistort"]:
                        contour_im = contourdetection(undistorted_im, params["contour_thresh"])
                    else:
                        contour_im = frame
                        contour_im = contourdetection(contour_im, params["contour_thresh"])
                    imgs["contours"] = contour_im

                if params["pingpong_color_threshed"]: # pingpong ball color thresholded
                    if params["undistort"]:
                        pingpong_im = colorThreshold(undistorted_im, "orange")
                    else:
                        pingpong_im = frame
                        pingpong_im = colorThreshold(pingpong_im, "orange")
                    imgs["pingpong_color_threshed"] = pingpong_im

                if params["boat_color_threshed"]: # boat color thresholded
                    if params["undistort"]:
                        boat_im = colorThreshold(undistorted_im, "white")
                    else:
                        boat_im = frame
                        boat_im = colorThreshold(boat_im, "white")
                    imgs["boat_color_threshed"] = boat_im

                displayResults(imgs)
                counter = 0
                key = cv2.waitKey(1)
            if key == ord('q'):
                break;
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    params = accept_json()
    #t0 = threading.Thread(target=keep_alive.connect_wifi, args=())
    t1 = threading.Thread(target=keep_alive.stream, args=())
    t2 = threading.Thread(target=main, args=(params,))
  
    # starting thread 0
    t0.start()
    sleep(30) # wait to establish connection
    # starting thread 1
    t1.start()
    # starting thread 2
    t2.start()
  
    # wait until thread 2 is completely executed
    t2.join()
    # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 0 is completely executed
    #t0.join()

    print("Done!")
