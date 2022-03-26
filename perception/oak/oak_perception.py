import cv2
import numpy as np
from time import time, sleep
from math import cos, pi
import json
import csv
from threading import Thread

from perception.oak.utils import *
from perception.oak.AR_tags import *
import perception.oak.plotting.custom_plotting as my_plt


"""
TODO:
    - This file might become deprecated or significantly changed with new camera
    - Integrate multi-boat localization
    - Optimize for speed and simplicity
    - Choose which data to pass on for generator functionality
"""


# AXES:
AXES = {'0': 'x',
        '1': 'y',
        '2': 'z'}

# ROOT:
ROOT = '.\\perception\\oak\\'

def accept_json(task):
    """
    Accepts json configuration file.
    """
    if task == "debug":
        with open(ROOT + 'configs\\debug_config.json', 'r') as f:
            PARAMS = json.load(f)
    elif task == "test":
        with open(ROOT + 'configs\\test_config.json', 'r') as f:
            PARAMS = json.load(f)
    return PARAMS;

def create_csv():
    """
    Creates csv to record data from the boats.
    """
    f = open(ROOT + 'recordings\\record.csv', 'w')
    writer = csv.writer(f)
    header = ['time', 'boom_cx', 'boom_cy', 'boat_cx', 'boat_cy', 'world_cx', 'world_cy', 'boom_theta', 'boat_theta']
    writer.writerow(header)
    return f, writer;

def im_process(params):
    """
    Main runner for image processing.  See utils for understanding function calls and
    the configs folder for understanding parameters.
    """
    if params["display"]:
        imgs = {}
        for key in params.keys():
            if params[key] == True and key != "display":
                imgs[key] = None
        
        for i, (window, value) in enumerate(params.items()):
            if value == True and window != "display":
                if window == "april":
                    if params["task"] == "test":
                        april = AR_PlaneDetection(False)
                    else:
                        april = AR_PlaneDetection(True)
                elif window == "undistort":
                    undistorter = UndistortionModule()
                if window != "undistort":
                    cv2.namedWindow(window)

    if params["task"] == "debug":
        # video pipeline
        cap = cv2.VideoCapture(ROOT + "test\\test_videos\\GX010115.MP4", cv2.CAP_FFMPEG)
    elif params["task"] == "test":
        # stream pipeline
        cap = cv2.VideoCapture(1, cv2.CAP_FFMPEG)
    
    t = time()

    while cap.isOpened():
        nmat, frame = cap.read()
        if nmat:
            if params["undistort"]: # camera lens undistortion
                undistorted_im = deepcopy(frame)
                undistorted_im = undistorter.undistort(undistorted_im)
                imgs["undistort"] = undistorted_im
            
            if params["april"]: # april tag detection
                april_im = deepcopy(frame)
                april_im, origins, ground_plane = april.detect_tags(april_im)
                tag_data = deepcopy(origins)
                all_states = april.computeAllStates(tag_data)
                if params["undistort"]:
                    april_im = deepcopy(undistorted_im)
                if params["display"]:
                    april_im = my_plt.plot_AR(april_im, tag_data, ground_plane)
                imgs["april"] = april_im

            if params["pingpong_color_threshed"]: # pingpong ball color thresholded
                if params["undistort"]:
                    pingpong_im = colorThreshold(undistorted_im, "orange")
                else:
                    pingpong_im = deepcopy(frame)
                    pingpong_im = colorThreshold(pingpong_im, "orange")
                imgs["pingpong_color_threshed"] = pingpong_im
            
            if params["display"]:
                my_plt.displayResults(imgs)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break;
            else:
                yield_dict = {"imgs": imgs, 
                              "state": all_states,
                              "origins": origins,
                              "ground": ground_plane}
                yield yield_dict
    cap.release()
    cv2.destroyAllWindows()

def launch(task):
    params = accept_json(task)
    t1 = Thread(target=im_process, args=(params,))
    t1.start()

if __name__ == "__main__":
    task = "debug"
    launch(task)