import cv2
import numpy as np
from time import time, sleep
import socket
from goprocam import GoProCamera
from goprocam import constants
from math import cos, pi
import random as rng
import json
import csv
import threading

from utils import *
from AR_tags import *
import keep_alive
import plotting.custom_plotting as my_plt


# AXES:
AXES = {'0': 'x',
        '1': 'y',
        '2': 'z'}

# ROOT:
ROOT = 'C:\\Schoolwork\\2022 Spring Semester\\Senior Design\\OSPREY_dev\\perception\\gopro'


def accept_json(task):
    """
    Accepts json configuration file.
    """
    if task == "debug":
        with open(ROOT + '\\configs\\debug_config.json', 'r') as f:
            PARAMS = json.load(f)
    elif task == "test":
        with open(ROOT + '\\configs\\test_config.json', 'r') as f:
            PARAMS = json.load(f)
    return PARAMS;

def create_csv():
    """
    Creates csv to record data from the boats.
    """
    f = open(ROOT + '\\recordings\\record.csv', 'w')
    writer = csv.writer(f)
    header = ['time', 'boat_cx', 'boat_cy', 'world_cx', 'world_cy', 'theta']
    writer.writerow(header)
    return f, writer;

def main(params):
    """
    Main runner for stream-based image processing.  See utils for understanding function calls and
    the configs folder for understanding parameters.
    """
    if params["display"]:
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
            if counter <= 0: # reduces effective frame rate
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
                    boat_pose, theta = april.computeState(origins)
                    if params["undistort"]:
                        april_im = undistorted_im
                    if params["display"]:
                        april_im = my_plt.plot_AR(april_im, origins, ground_plane)
                        imgs["april"] = april_im
                    
                if params["edges"] and not params["contours"]: # edge detection
                    if params["undistort"]:
                        edge_im = edgedetection(undistorted_im, params["contour_thresh"])
                    else:
                        edge_im = frame
                        edge_im = edgedetection(edge_im, params["contour_thresh"])
                    if params["display"]:
                        imgs["edges"] = edge_im

                if params["contours"]: # contour detection
                    if params["undistort"]:
                        contour_im = contourdetection(undistorted_im, params["contour_thresh"])
                    else:
                        contour_im = frame
                        contour_im = contourdetection(contour_im, params["contour_thresh"])
                    if params["display"]:
                        imgs["contours"] = contour_im

                if params["pingpong_color_threshed"]: # pingpong ball color thresholded
                    if params["undistort"]:
                        pingpong_im = colorThreshold(undistorted_im, "orange")
                    else:
                        pingpong_im = frame
                        pingpong_im = colorThreshold(pingpong_im, "orange")
                    if params["display"]:
                        imgs["pingpong_color_threshed"] = pingpong_im

                if params["boat_color_threshed"]: # boat color thresholded
                    if params["undistort"]:
                        boat_im = colorThreshold(undistorted_im, "white")
                    else:
                        boat_im = frame
                        boat_im = colorThreshold(boat_im, "white")
                    if params["display"]:
                        imgs["boat_color_threshed"] = boat_im
                
                if params["display"]:
                    my_plt.displayResults(imgs)
                
                counter = 0
                key = cv2.waitKey(1)
            if key == ord('q'):
                break;
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    params = accept_json("test")
    # t0 = threading.Thread(target=keep_alive.connect_wifi, args=())
    t1 = threading.Thread(target=keep_alive.stream, args=())
    t2 = threading.Thread(target=main, args=(params,))
  
    # starting thread 0
    # t0.start()
    # sleep(30) # wait to establish connection
    # starting thread 1
    t1.start()
    # starting thread 2
    t2.start()
  
    # wait until thread 2 is completely executed
    t2.join()
    # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 0 is completely executed
    # t0.join()

    print("Done!")
