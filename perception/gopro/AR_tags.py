import threading
from itertools import product
import cv2
from pupil_apriltags import Detector
from math import acos, sqrt

from utils import *


class AR_PlaneDetection(object):
    def __init__(self, rows=5, cols=7):
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
        self.__world_origin = None
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
            for i, tag in enumerate(tags):
                if self.__world_origin is None: # first define the world coordinate system
                    if tag.tag_id == 27:
                        center_pt = tag.center
                        center_pt = center_pt.astype(int)
                        center_pt = (center_pt[0], center_pt[1])
                        p = self.find_ground_plane(tag)
                        xy_origin = self.find_tag_origin(tag, p)
                        xy_origin.insert(0, center_pt)
                        tag_data["ground"] = (xy_origin, tag.pose_R, tag.pose_t)
                        self.__world_origin = xy_origin
                else:
                    center_pt = tag.center
                    center_pt = center_pt.astype(int)
                    center_pt = (center_pt[0], center_pt[1])
                    if tag.tag_id == 27:
                        xy_origin = self.__world_origin
                        tag_data["ground"] = (xy_origin, tag.pose_R, tag.pose_t) # dont change ground plane
                    else:
                        xy_origin = self.find_tag_origin(tag)
                        xy_origin.insert(0, center_pt)
                        tag_data["boat"] = (xy_origin, tag.pose_R, tag.pose_t)
        gridworld = self.__2d_pts
        return src, tag_data, gridworld;

    def find_tag_origin(self, tag, p=None):
        """
        Create axes for a tag, stored as a 2x2 numpy array (x, y) (pixel coordinates)
        """
        xyz_world = [(1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)]
        if p is None:
            p = get_transformation(self.__K, tag.pose_R, tag.pose_t)
        xy_origin = convert_3d_to_2d(p, xyz_world)
        return xy_origin;


    def ground2boat(self, tag_data):
        """
        Overlay the ground origin onto the boat origin

        TODO: Change this function to use composed Rotation Matrices to compute angle
        if we register boat and ground tags, then we have (R_cw, t_cw) and (R_cb, t_cb)
        where R_wc, R_bc are the rotations from tags to camera (world and boat, respectively)
        We can compute R_wb, which is the rotation from world to boat by R_wb = R_wc @ R_wb.T
        """
        ground_origin_overlay = None
        if "boat" in tag_data.keys() and "ground" in tag_data.keys():
            boat_origin = tag_data["boat"][0]
            ground_origin = tag_data["ground"][0]
            translation = (boat_origin[0][0] - ground_origin[0][0], boat_origin[0][1] - ground_origin[0][1])
            ground_origin_overlay = [];
            for i, pt in enumerate(ground_origin):
                temp_x = ground_origin[i][0] + translation[0]
                temp_y = ground_origin[i][1] + translation[1]
                ground_origin_overlay.append((temp_x, temp_y))
        return ground_origin_overlay;

    def calculateHeading(self, tag_data):
        """
        Compute Heading Angle of the boat relative wrt world coordinate system and the y-distance of the boat to the world origin
        Note: this funtion is done in the image plane
        """
        ground_origin_overlay = self.ground2boat(tag_data)
        theta = None
        if ground_origin_overlay is not None:
            boat_origin = tag_data["boat"][0]
            world_origin = tag_data["ground"][0]
            boat_x = (boat_origin[2][0] - boat_origin[0][0], boat_origin[2][1] - boat_origin[0][1])
            world_x = (ground_origin_overlay[2][0] - ground_origin_overlay[0][0], ground_origin_overlay[2][1] - ground_origin_overlay[0][1])
            # use dot product to compute angle
            dot_prod = boat_x[0]*world_x[0] + boat_x[1]*world_x[1]
            cos_theta = dot_prod / (sqrt(boat_x[0]**2 + boat_x[1]**2)*sqrt(world_x[0]**2 + world_x[1]**2))
            theta = acos(max(-1.0, min(1.0, cos_theta)))
            # distance along the y-axis of the world coordinates corresponds to x-axis of the image
            distance = boat_origin[0] - world_origin[0]
        return ground_origin_overlay, theta;


    def computeState(self, tag_data):
        """
        Compute the Heading and Offset of the boat wrt the world coordinate system.
        Note: this function is done in the world coordinate system using rotation matrices
        """
        theta = None
        distance = None
        if "boat" in tag_data.keys() and "ground" in tag_data.keys():
            # boat wrt camera
            R_bc = tag_data["boat"][1]
            t_bc = tag_data["boat"][2]
            # world wrt camera
            R_wc = tag_data["ground"][1]
            t_wc = tag_data["ground"][2]
            # boat wrt world
            R_bw = R_bc @ R_wc.T
            t_bw = t_bc - t_wc
            # y axis of the world wrt to world
            y_ww = (0.0, 1.0, 0.0)
            # y axis of the boat wrt the world
            y_bw = (R_bw @ y_ww) + t_bw
            # theta angle -> heading of the boat in the world
            cos_theta = y_bw[1] / np.linalg.norm(y_bw)
            theta = acos(max(-1.0, min(1.0, cos_theta)))
            # distance
            distance = t_bw[1]
        return distance, theta;
