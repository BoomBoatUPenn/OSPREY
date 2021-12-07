import cv2
import numpy as np
from time import time
import socket
from goprocam import GoProCamera
from goprocam import constants
from math import cos, pi
import random as rng


from utils import *
from AR_tags import *

# Calibrated GoPro Camera Parameters (OpenCV)
#K = np.array([[1425.5513881614268, 0.0, 1654.2719012237355], 
#              [0.0, 1425.3478800534128, 1197.3579471903774], 
#              [0.0, 0.0, 1.0]])
#D = np.array([[0.09193360142523685], [0.14517625987376206], [-0.4878419565489222], [0.4019373729476832]])

# Calibrated GoPro Camera Parameters (MATLAB)
#K = np.array([[1224., 0.0, 3.6242e3], 
#              [0.0, 1224., 2.5663e3], 
#              [0.0, 0.0, 1.0]])
#D = np.array([[1.3975e3], [-1.3595e-4], [-3.0898e-8], [4.0363e-13]])
# MATLAB derived camera params for pinhole camera model ([fx, fy, cx, cy])
#camera_params = [1224., 1224., 3.6242e3, 2.5663e3]


def stream_cap(augmented_reality, record, make_contours, val):
    undistorter = UndistortionModule()
    if augmented_reality:
        plane_detector = AR_PlaneDetection()
    gpCam = GoProCamera.GoPro()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gpCam.livestream("start")
    gpCam.video_settings(res='1080p', fps='12')
    gpCam.gpControlSet(constants.Stream.BIT_RATE, constants.Stream.BitRate.B2_4Mbps)
    cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)
    if record:
        out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (240,432))
    t = time()
    if make_contours:
        cv2.namedWindow('OpenCV (Contours)')
    else:
        cv2.namedWindow('OpenCV (Edges)')
    cv2.namedWindow('Undistorted')
    counter = 0
    while cap.isOpened():
        nmat, frame = cap.read()
        if nmat:
            if counter <= 3:
                counter += 1
                key = cv2.waitKey(15)
            else:
                undistorted = undistorter.undistort(frame)
                if augmented_reality:
                    pts_2d = plane_detector.detect_AprilTag(undistorted)
                    if pts_2d is not None:
                        for pt in pts_2d:
                            cv2.circle(undistorted, pt, 3, (0, 0, 255), -1)
                cv2.imshow('Undistorted', undistorted)

                if make_contours:
                    processed = contourdetection(undistorted, val)
                    cv2.imshow('OpenCV (Contours)', processed)
                else:
                    processed = edgedetection(undistorted, val)
                    cv2.imshow('OpenCV (Edges)', processed)
                if record:
                    out.write(processed)
                key = cv2.waitKey(1)
                counter = 0
            if key == ord('q'):
                break;
            if time() - t >= 2.5:
                sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
                t=time()
        else:
            break;
    # When everything is done, release the capture
    cap.release()
    if record:
        out.release()
    cv2.destroyAllWindows()

augmented_reality = False
record = False
make_contours = False
val = 20
stream_cap(augmented_reality, record, make_contours, val)
