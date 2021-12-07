import cv2
import numpy as np
from typing import List, Tuple, Set, Optional

# Utility functions for main perception module

def edgedetection(src, val):
    src_gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    src_gray = cv2.blur(src_gray, (3,3))
    threshold = val
    canny_output = cv2.Canny(src_gray, threshold, threshold * 2)
    return canny_output;


def contourdetection(src, val):
    src_gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    src_gray = cv2.blur(src_gray, (3,3))
    threshold = val
    canny_output = cv2.Canny(src_gray, threshold, threshold * 2)
    contours, hierarchy = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        cv2.drawContours(drawing, contours, i, color, 2, cv2.LINE_8, hierarchy, 0)
    return drawing;

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
        self.K = np.array([[1425.5513881614268, 0.0, 1654.2719012237355], 
                           [0.0, 1425.3478800534128, 1197.3579471903774], 
                           [0.0, 0.0, 1.0]])
        self.D = np.array([[0.09193360142523685], [0.14517625987376206], [-0.4878419565489222], [0.4019373729476832]])
        self.DIM = (3264, 2448)
        self.balance = 1
        self.crop = 0.1
        self.resize_factor = 1.0
        return;

    def undistort(self, src):
        dims = src.shape[:2][::-1]
        dims1 = dims
        dims2 = dims
        scaled_K = self.K* dims[0] / self.DIM[0]  # The values of K is to scale with image dimension.
        scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, self.D, dims1, np.eye(3), balance=self.balance)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, self.D, np.eye(3), new_K, dims2, cv2.CV_16SC2)
        undistorted_img = cv2.remap(src, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
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
    for row in range(n-1):
        for col in range(m-1):
            b_box = (maze_in_2d[row][col], maze_in_2d[row+1][col+1])
            if x_point >= b_box[0][0] and x_point < b_box[1][0]:
                if y_point >= b_box[0][1] and y_point < b_box[1][1]:
                    return (row, col);
