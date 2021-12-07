import threading
from itertools import product
import cv2
from pupil_apriltags import Detector

from utils import *


class AR_PlaneDetection(object):
    def __init__(self, rows=10, cols=10):
        row_lim = rows // 2
        col_lim = cols // 2
        self.__camera_params = [1224., 1224., 3.6242e3, 2.5663e3]
        self.__K = np.array([[1224., 0.0, 3.6242e3], 
                             [0.0, 1224., 2.5663e3],
                             [0.0, 0.0, 1.0]])
        self.__rows = rows
        self.__cols = cols
        self.__mouse_pos = (0, 0)
        self.__grids = list(product(range(row_lim - rows, row_lim + 1), range(col_lim - cols, col_lim + 1), (0,)))
        self.__2d_pts = None
        self.__obstacles = set()
        self.__start = None
        self.__goals = set()
        self.__path = None
        self.__thread = None
        self.at_detector = Detector('tag36h11')

    def __mouse_event(self, event, x, y, *_):
        coord = (x, y)
        if event == cv2.EVENT_MOUSEMOVE:
            self.__mouse_pos = coord
        if self.__2d_pts and not self.__thread:
            if event == cv2.EVENT_LBUTTONDOWN:
                relative = convert_2d_to_relative(coord, self.__reshaped_pts)
                if relative not in self.__obstacles:
                    self.__start = relative
                    self.__path = None
            elif event == cv2.EVENT_RBUTTONDOWN:
                relative = convert_2d_to_relative(coord, self.__reshaped_pts)
                if relative not in self.__obstacles:
                    if relative and relative not in self.__goals:
                        self.__goals.add(relative)
                    else:
                        self.__goals.discard(relative)
                    self.__path = None

    def __get_xy(self, x, y):
        return self.__2d_pts[x * (self.__cols + 1) + y]

    def __xy_to_world(self, x, y):
        return x - self.__rows + self.__rows // 2 + .5, y - self.__cols + self.__cols // 2 + .5, 0

    @property
    def __reshaped_pts(self):
        return [
            [self.__2d_pts[i * (self.__cols + 1) + j] for j in range(self.__cols + 1)] for i in range(self.__rows + 1)]
    
    def detect_AprilTag(self, src):
        tags = self.at_detector.detect(cv2.cvtColor(src, cv2.COLOR_BGR2GRAY), estimate_tag_pose=True, camera_params=self.__camera_params, tag_size=0.2)
        self.__2d_pts = None
        if len(tags) != 0:
            tag = tags[0]
            p = get_transformation(self.__K, tag.pose_R, tag.pose_t)
            self.__2d_pts = convert_3d_to_2d(p, self.__grids)
        return self.__2d_pts;
