import cv2
import numpy as np
from time import time
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
        if window == "april":
            if value == True:
                april = AR_PlaneDetection()
        if window == "undistort":
            if value == True:
                undistorter = UndistortionModule()
        if value == True:
            cv2.namedWindow(window)

    cap = cv2.VideoCapture("udp://127.0.0.1:10000")
    counter = 0
    t = time()
    while cap.isOpened():
        nmat, frame = cap.read()
        if nmat:
            if counter <= 4: # reduces effective frame rate
                counter += 1
                key = cv2.waitKey(1)
            else:
                if params["undistort"]: # camera lens undistortion
                    undistorted_im = frame
                    undistorted_im = undistorter.undistort(undistorted_im)
                    imgs["undistort"] = undistorted_im
                
                if params["april"]: # april tag detection
                    april_im = frame
                    april_im, origins, ground_plane = april.detect_tags(april_im)
                    if params["undistort"]:
                        april_im = undistorter.undistort(april_im)
                    if "ground" in origins.keys():
                        for pt in origins["ground"][0][1:]:
                            cv2.arrowedLine(april_im, origins["ground"][0][0], pt, (0, 255, 0))
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

    t1 = threading.Thread(target=keep_alive.stream, args=())
    t2 = threading.Thread(target=main, args=(params,))
  
    # starting thread 1
    t1.start()
    # starting thread 2
    t2.start()
  
    # wait until thread 2 is completely executed
    t2.join()
    # wait until thread 1 is completely executed
    t1.join()
    
    # both threads completely executed
    print("Done!")
