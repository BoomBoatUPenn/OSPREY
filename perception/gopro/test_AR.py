import cv2
import numpy as np
from math import cos, pi

from utils import *
from AR_tags import *


ROOT = ".\perception\\gopro\\test\\test_images\\"
# LEFT or RIGHT
left = {}
right = {}
# ADJACENT, PARALLEL, CCW, CW
left["adjacent"] = "GOPR0095.JPG"
left["parallel"] = "GOPR0099.JPG"
left["ccw"] = "GOPR0103.JPG"
left["cw"] = "GOPR0107.JPG"

right["adjacent"] = "GOPR0097.JPG"
right["parallel"] = "GOPR0101.JPG"
right["ccw"] = "GOPR0105.JPG"
right["cw"] = "GOPR0109.JPG"

images = {"left": left,
          "right": right}

def AR_photo_test():
    key = None
    frame = cv2.imread(ROOT + images["right"]["ccw"])
    dims = frame.shape[:2][::-1]
    resize_factor = 1./4.
    cv2.namedWindow("April")
    AXES = {'0': 'x',
            '1': 'y',
            '2': 'z'}

    april = AR_PlaneDetection()
    undistorter = UndistortionModule()

    april_im = frame
    resized_shape = (int(dims[0]*resize_factor), int(dims[1]*resize_factor))

    april_im = cv2.resize(april_im, dsize=resized_shape)
    april_im, origins, ground_plane = april.detect_tags(april_im, set_K=True)
    tag_data = deepcopy(origins)
    distance, theta = april.computeState(tag_data)
    print("Distance: ", distance, ", Theta: ", theta)
    undistorted = undistorter.undistort(frame)
    april_im = undistorter.undistort(frame)

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
        for row in ground_plane:
            for pt in row:
                cv2.circle(april_im, pt, 3, (0, 0, 255), -1)
        ground_c = origins["ground"][0][0]
        cv2.circle(april_im, ground_c, 3, (255, 0, 122), -1)
    while key != ord('q'):
        cv2.imshow("April", april_im)
        key = cv2.waitKey(1)

if __name__ == "__main__":
    AR_photo_test()