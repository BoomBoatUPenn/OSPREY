import threading
from itertools import product
import cv2
from pupil_apriltags import Detector

from utils import *


class AR_PlaneDetection(object):
    def __init__(self, rows=3, cols=5):
        row_lim = rows // 2
        col_lim = cols // 2
        self.__camera_params = [1889.22664838299, 1880.09363713027, 1940.53184414707, 1088.10395505467]
        self.__K = np.array([[1889.22664838299, 0.0, 1940.53184414707], 
                             [0.0, 1880.09363713027, 1088.10395505467],
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
        self.at_detector = Detector(families='tag36h11',
                                    nthreads=2,
                                    quad_decimate=2.0,
                                    quad_sigma=0.1,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

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
    
    def find_ground_plane(self, tag):
        """
        Find ground plane for static tag (family tag36h11)
        """
        self.__2d_pts = None
        p = get_transformation(self.__K, tag.pose_R, tag.pose_t)
        self.__2d_pts = convert_3d_to_2d(p, self.__grids)
        return p;

    def detect_tags(self, src):
        """
        Detect all tags and call helper functions for static and moving tags.
        """
        tags = self.at_detector.detect(cv2.cvtColor(src, cv2.COLOR_BGR2GRAY), estimate_tag_pose=True, camera_params=self.__camera_params, tag_size=0.14263)
        tag_data = {}
        if len(tags) != 0:
            for tag in tags:
                if str(tag.tag_family) == "b'tag36h11'": # use this tag family for ground plane
                    p = self.find_ground_plane(tag)
                    xy_origin = self.find_tag_origin(tag, p)
                    center_pt = tag.center
                    center_pt = center_pt.astype(int)
                    center_pt = (center_pt[0], center_pt[1])
                    xy_origin.insert(0, center_pt)
                    tag_data["ground"] = (xy_origin, tag.pose_R, tag.pose_t)
                else: # other tag is used for the boat pose
                    xy_origin = self.find_tag_origin(tag)
                    center_pt = tag.center
                    center_pt = center_pt.astype(int)
                    center_pt = (center_pt[0], center_pt[1])
                    xy_origin.insert(0, center_pt)
                    tag_data["boat"] = (xy_origin, tag.pose_R, tag.pose_t)
        return src, tag_data, self.__2d_pts;

    def find_tag_origin(self, tag, p=None):
        """
        create axes for a tag, stored as a 2x2 numpy array (x, y) (pixel coordinates)
        """
        xyz_world = [(0.25, 0.0, 0.0), (0.0, 0.25, 0.0)]
        if p is None:
            p = get_transformation(self.__K, tag.pose_R, tag.pose_t)
        xy_origin = convert_3d_to_2d(p, xyz_world)
        return xy_origin;


    def ground2boat(self, tag_data):
        ground_origin_overlay = None
        if "boat" in tag_data.keys() and "ground" in tag_data.keys():
            boat_origin = tag_data["boat"][0]
            ground_origin = tag_data["ground"][0]
            translation = (boat_origin[0][0] - ground_origin[0][0], boat_origin[0][1] - ground_origin[0][1])
            ground_origin_overlay = ground_origin;
            for i, pt in enumerate(ground_origin):
                ground_origin_overlay[i][0] += translation[0]
                ground_origin_overlay[1][1] += translation[1]
        return ground_origin_overlay;

        # R_ground2boat = None
        # if "boat" in tag_data.keys() and "ground" in tag_data.keys():
        #     R_boat = tag_data["boat"] # rotation of boat wrt to camera
        #     R_ground = tag_data["ground"] # rotation of ground wrt to camera
        #     R_ground2boat = R_ground @ R_boat.T # rotation of ground wrt to boat
        # return R_ground2boat;