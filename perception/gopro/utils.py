import cv2
import numpy as np
import random as rng
from typing import List, Tuple, Set, Optional
from collections import deque

# Utility functions for main perception module

def colorThreshold(src, color):
    if color == "orange": # for balls
        lower_color_bounds = (0, 125, 100)
        upper_color_bounds = (170, 255, 255)
    elif color == "white": # for boats
        lower_color_bounds = (105, 115, 150)
        upper_color_bounds = (200, 240, 255)
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(src, lower_color_bounds, upper_color_bounds)
    mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    res = src & mask_rgb
    return res;

def edgedetection(src, val):
    src_gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    src_gray = cv2.blur(src_gray, (3,3))
    canny_output = cv2.Canny(src_gray, val, val * 2)
    return canny_output;

def contourdetection(src, val):
    threshold = val
    canny_output = edgedetection(src, threshold)
    contours, hierarchy = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        cv2.drawContours(drawing, contours, i, color, 2, cv2.LINE_8, hierarchy, 0)
    return drawing;

def balldetection(src, template):
    w, h = template.shape[::-1]
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.55
    loc = np.where(res >= threshold)
    counter = 0
    for pt in zip(*loc[::-1]):
        if counter > 20:
            break;
        cv2.rectangle(src, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
        counter += 1
    return src;

def blobdetection(src, detector):
    #gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    keypoints = detector.detect(src)
    im_with_keypoints = cv2.drawKeypoints(src, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return im_with_keypoints;

def gridlines(frame):
    h, w, _ = frame.shape
    rows, cols = (10, 10)
    dy, dx = h / rows, w / cols
    color = (0, 255, 0)
    thickness = 1
    for col in np.linspace(start=dx, stop=w-dx, num=cols-1):
        col = int(round(col))
        cv2.line(frame, (col, 0), (col, h), color=color, thickness=thickness)

    for row in np.linspace(start=dy, stop=h-dy, num=rows-1):
        row = int(round(row))
        cv2.line(frame, (0, row), (w, row), color=color, thickness=thickness)
    return frame;

class UndistortionModule(object):
    def __init__(self, *args, **kwargs):
        self.K = np.array([[1889.22664838299, 0.0, 1940.53184414707], 
                             [0.0, 1880.09363713027, 1088.10395505467],
                             [0.0, 0.0, 1.0]])
        self.D = np.array([[-0.265599284726457], [0.121174981668003], [0.0], [0.0], [-0.0354316026650457]])
        self.DIM = (3840, 2160)
        self.balance = 1
        self.crop = 0.0
        self.resize_factor = 1./4.

    def undistort(self, src):
        dims = src.shape[:2][::-1]
        scaled_K = self.K* dims[0] / self.DIM[0]  # The values of K is to scale with image dimension.
        scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        undistorted_img = cv2.undistort(src, scaled_K, self.D, None)
        if self.crop > 0:
            crop_offset = (int(self.crop*dims[0]), int(self.crop*dims[1]))
            cropped_undistorted_img = undistorted_img[crop_offset[0]:-crop_offset[0], crop_offset[1]:-crop_offset[1], :]
            dims = cropped_undistorted_img.shape[:2][::-1]
            undistorted_img = cropped_undistorted_img
        resized_shape = (int(dims[0]*self.resize_factor), int(dims[1]*self.resize_factor))
        resized_undistorted_img = cv2.resize(undistorted_img, dsize=resized_shape)
        return resized_undistorted_img;


Vertex = Tuple[int, int]
Edge = Tuple[Vertex, Vertex]


def get_transformation(k: np.ndarray, r: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Calculate the transformation matrix using the given equation P = K x (R | T)"""
    r_t = np.hstack((r, t))
    p = np.matmul(k, r_t)
    return p;


def convert_3d_to_2d(p: np.ndarray, points_3d: List[Tuple[float, float, float]]) -> List[Tuple[int, int]]:
    """Convert a list of 3D real world points to 2D image points in pixels given the transformation matrix,
       preserving the order of the points."""
    points_2d = []
    for point_3d in points_3d:
        x, y, z = point_3d
        modded = np.array([x, y, z, 1.0])
        point = np.matmul(p, modded)
        new_z = point[2]
        point = (point / new_z).astype(int)
        points_2d.append(tuple(point[:2]))
    return points_2d;


def convert_2d_to_relative(point_2d: Tuple[int, int], maze_in_2d: List[List[Tuple[int, int]]]) -> Optional[Vertex]:
    """Convert a 2D image point to maze coordinates using the given maze coordinates in 2D image.
       Return None if the 2D point isn't in the maze. Assume the coordinates are axis-aligned."""
    n = len(maze_in_2d)
    m = len(maze_in_2d[0])
    x_point, y_point = point_2d
    r = None
    c = None
    min_dist = 1e10
    for row in range(n):
        for col in range(m):
            maze_pt = maze_in_2d[row][col]
            if maze_pt[0] > 0 and maze_pt[1] > 0:
                l2_dist = ((point_2d[0] - maze_pt[0]) / 10000)**2 + ((point_2d[1] - maze_pt[1]) / 10000)**2
                if l2_dist < min_dist:
                    min_dist = l2_dist
                    r, c = row, col
    return r, c;