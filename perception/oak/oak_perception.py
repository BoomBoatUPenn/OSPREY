import cv2
import numpy as np
from time import time, sleep
from math import cos, pi
import json
import csv
from threading import Thread
from copy import deepcopy

from perception.oak.utils import *
from perception.oak.AR_tags import *
import perception.oak.plotting.custom_plotting as my_plt
from perception.oak.oak_pipeline import OAKPipeline


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
    return PARAMS

def create_csv():
    """
    Creates csv to record data from the boats.
    """
    f = open(ROOT + 'recordings\\record.csv', 'w')
    writer = csv.writer(f)
    header = ['time', 'boom_cx', 'boom_cy', 'boat_cx', 'boat_cy', 'world_cx', 'world_cy', 'boom_theta', 'boat_theta']
    writer.writerow(header)
    return f, writer

def im_process(params):
    """
    Main runner for image processing.  See utils for understanding function calls and
    the configs folder for understanding parameters.
    """

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
            if window != "undistort" and params["display"]:
                cv2.namedWindow(window)

    # oak pipeline
    cap = OAKPipeline()
    cap.startDevice()
    sleep(5)
    
    t = time()
    yield_dict = {"imgs": None, 
                "state": None,
                "origins": None,
                "ground": None}

    while cap.isOpened():
        frame_dict = cap.read()
        if "rgb" in frame_dict.keys():
            rgb_frame = frame_dict["rgb"]
        if "depth" in frame_dict.keys():
            depth_frame = frame_dict["depth"]
        if rgb_frame is not None:
            if params["april"]: # april tag detection
                april_im = deepcopy(rgb_frame)
                _, origins, ground_plane = april.detect_tags(april_im)
                all_states = april.computeAllStates(origins)
                if params["display"]:
                    april_im = my_plt.plot_AR(april_im, origins, ground_plane)
                imgs["april"] = april_im

            if params["pingpong_color_threshed"]: # pingpong ball color thresholded
                pingpong_im = rgb_frame
                pingpong_im = colorThreshold(pingpong_im, "orange")
                imgs["pingpong_color_threshed"] = pingpong_im
            
            if params["display"]:
                my_plt.displayResults(imgs)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            else:
                yield_dict["imgs"] = imgs, 
                yield_dict["state"] = all_states
                yield_dict["origins"] = origins
                yield_dict["ground"] = ground_plane

                yield yield_dict
    #cap.release()
    cv2.destroyAllWindows()

def launch(task):
    params = accept_json(task)
    t1 = Thread(target=im_process, args=(params,))
    t1.start()

if __name__ == "__main__":
    task = "debug"
    launch(task)