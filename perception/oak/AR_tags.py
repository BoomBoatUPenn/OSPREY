import threading
from itertools import product
import cv2
from pupil_apriltags import Detector
from math import atan2, acos

from perception.oak.utils import *

"""
TODO:
    - This class may be deprecated with a new camera
    - Integrate multi-boat tag detection and localization
    - Calibrate Oak-D Camera (maybe)
"""

class AR_PlaneDetection(object):
    """
    Main class for AR World and Boat perception, used for localization and mapping.
    """
    def __init__(self, sim, resolution=0.1, rows=5, cols=5):
        row_lim = rows // 2
        col_lim = cols // 2
        # GOPRO PARAMS
        """
        self.__camera_params = [1889.22664838299, 1880.09363713027, 1940.53184414707, 1088.10395505467]
        self.__K = np.array([[1889.22664838299, 0.0, 1940.53184414707], 
                             [0.0, 1880.09363713027, 1088.10395505467],
                             [0.0, 0.0, 1.0]])
        """
        self.__camera_params = [3098.163330078125, 3098.163330078125, 1945.3646240234375, 1092.97509765625]
        self.__K = [[3098.163330078125, 0.0, 1945.3646240234375], 
                    [0.0, 3098.163330078125, 1092.97509765625], 
                    [0.0, 0.0, 1.0]]
        self.__DIMS = (3840, 2160)
        self.__resolution = resolution
        self.__rows = rows
        self.__cols = cols
        self.__grids = list(product(np.arange(row_lim - rows, row_lim, self.__resolution), np.arange(col_lim - cols, col_lim, self.__resolution), (0,)))
        self.__world_origin = None
        self.__2d_pts = None
        self.__sim = sim
        if sim:
            self.at_detector = Detector(families='tag36h11', 
                                        nthreads=4,
                                        quad_decimate=3.0,
                                        quad_sigma=0.1,
                                        refine_edges=1,
                                        decode_sharpening=0.25,
                                        debug=0)
        else:
            self.at_detector = Detector(families='tag36h11', 
                                        nthreads=4,
                                        quad_decimate=3.0,
                                        quad_sigma=0.0,
                                        refine_edges=1,
                                        decode_sharpening=0.1,
                                        debug=0)


    def __get_xy(self, x, y):
        return self.__2d_pts[x * (self.__cols + 1) + y];

    def __xy_to_world(self, x, y):
        return x - self.__rows + self.__rows // 2 + .5, y - self.__cols + self.__cols // 2 + .5, 0;

    @property
    def __reshaped_pts(self):
        return [
            [self.__2d_pts[i * int(self.__cols / self.__resolution) + j] for j in range(int(self.__cols / self.__resolution))] for i in range(int(self.__rows / self.__resolution))];
    
    def find_ground_plane(self, tag):
        """
        Find ground plane for static tag
        """
        self.__2d_pts = None
        p = get_transformation(self.__K, tag.pose_R, tag.pose_t)
        self.__2d_pts = convert_3d_to_2d(p, self.__grids)
        return p;

    def detect_tags(self, src, set_K=False):
        """
        Detect all tags and call helper functions for static and moving tags
        """
        if set_K:
            dims = src.shape[:2][::-1]
            scaled_K = self.__K * dims[0] / self.__DIMS[0]  # The values of K is to scale with image dimension.
            scaled_params = [i * dims[0] / self.__DIMS[0] for i in self.__camera_params]
            scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
            self.__K = scaled_K
            self.__camera_params = scaled_params
        tags = self.at_detector.detect(cv2.cvtColor(src, cv2.COLOR_BGR2GRAY), estimate_tag_pose=True, camera_params=self.__camera_params, tag_size=0.15)
        tag_data = {}
        if len(tags) != 0:
            for i, tag in enumerate(tags):
                if tag.tag_id == 27: # Ground
                    if self.__world_origin is None: # first define the world coordinate system
                        center_pt = tag.center
                        center_pt = center_pt.astype(int)
                        center_pt = (center_pt[0], center_pt[1])
                        p = self.find_ground_plane(tag)
                        xy_origin = self.find_tag_origin(tag, p)
                        xy_origin.insert(0, center_pt)
                        tag_data["ground"] = (xy_origin, tag.pose_R, tag.pose_t)
                        self.__world_origin = xy_origin
                    else:
                        xy_origin = self.__world_origin
                        tag_data["ground"] = (xy_origin, tag.pose_R, tag.pose_t) # dont change ground plane
                elif tag.tag_id == 54: # Boom
                    center_pt = tag.center
                    center_pt = center_pt.astype(int)
                    center_pt = (center_pt[0], center_pt[1])
                    xy_origin = self.find_tag_origin(tag)
                    xy_origin.insert(0, center_pt)
                    tag_data["boom"] = (xy_origin, tag.pose_R, tag.pose_t)
                else: # Boat, tag.tag_id == 0
                    center_pt = tag.center
                    center_pt = center_pt.astype(int)
                    center_pt = (center_pt[0], center_pt[1])
                    xy_origin = self.find_tag_origin(tag)
                    xy_origin.insert(0, center_pt)
                    tag_data["boat"] = (xy_origin, tag.pose_R, tag.pose_t)
        gridworld = None
        if self.__2d_pts is not None:
            gridworld = self.__reshaped_pts
        return src, tag_data, gridworld;

    def find_tag_origin(self, tag, p=None):
        """
        Create axes for a tag, stored as a 2x2 numpy array (x, y) (pixel coordinates)
        """
        xyz_world = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        if p is None:
            p = get_transformation(self.__K, tag.pose_R, tag.pose_t)
        xy_origin = convert_3d_to_2d(p, xyz_world)
        return xy_origin;

    def computeState(self, R_wc, boomboat):
        """
        DEPRECATED
        Compute the Heading and Offset of boom or boat wrt the world coordinate system.
        Note: this function is done in the world coordinate system
        """
        theta = None
        pose = None
        # boat wrt camera
        R_bc = boomboat[1]
        # boat wrt world
        R_bw = (R_bc @ R_wc.T).T
        R_bw[:, -1] = np.cross(R_bw[:, 0], R_bw[:, 1])
        # y axis of the world wrt to world
        y_ww = np.array([[1.0, 0.0, 0.0]]).T
        # y axis of the boat wrt the world
        y_ww_rot = R_bw @ y_ww
        # theta angle -> heading of the boat in the world
        theta_J1 = atan2(-1.0*y_ww_rot[0, 0], y_ww_rot[1, 0])
        theta_J2 = atan2(y_ww_rot[0, 0], y_ww_rot[1, 0])
        theta_J3 = atan2(y_ww_rot[0, 0], -1.0*y_ww_rot[1, 0])
        theta_J4 = atan2(-1.0*y_ww_rot[0, 0], -1.0*y_ww_rot[1, 0])
        #theta_A = acos(np.dot(y_ww, y_ww_rot.T) / (np.linalg.norm(y_ww) * np.linalg.norm(y_ww_rot)))
        cos_theta = np.dot(y_ww.T, y_ww_rot) / (np.linalg.norm(y_ww) * np.linalg.norm(y_ww_rot))
        b_grid = convert_2d_to_relative(boomboat[0][0], self.__reshaped_pts)
        w_grid = (int(self.__rows / self.__resolution), int(self.__cols / self.__resolution))
        if b_grid is not None:
            r_b, c_b = b_grid
            x_b = r_b * self.__resolution
            y_b = c_b * self.__resolution
            r_w, c_w = w_grid
            x_w = r_w * self.__resolution
            y_w = c_w * self.__resolution
            pose = (x_b - x_w, y_b - y_w)
        return pose, theta;


    def computeStateV2(self, c_world, c_boomboat):
        theta = None
        pose = None
        c_w_x = (convert_2d_to_relative(c_world[0], self.__reshaped_pts), convert_2d_to_relative(c_world[1], self.__reshaped_pts))
        c_b_x = (convert_2d_to_relative(c_boomboat[0], self.__reshaped_pts), convert_2d_to_relative(c_boomboat[1], self.__reshaped_pts))
        c_w_x_vector = np.array([[c_w_x[0][0] - c_w_x[0][0], c_w_x[0][1] - c_w_x[0][1]], 
                                [c_w_x[1][0] - c_w_x[0][0], c_w_x[1][1] - c_w_x[0][1]]], dtype=np.float64)
        c_b_x_vector = np.array([[c_b_x[0][0] - c_w_x[0][0], c_b_x[0][1] - c_w_x[0][1]], 
                                [c_b_x[1][0] - c_w_x[0][0], c_b_x[1][1] - c_w_x[0][1]]], dtype=np.float64)
        c_w_x_vector *= self.__resolution
        c_b_x_vector *= self.__resolution
        pose = tuple(c_b_x_vector[0])
        c_b_x_dottable = c_b_x_vector[1] - c_b_x_vector[0]
        cos_theta = np.dot(c_b_x_dottable.T, c_w_x_vector[1]) / (np.linalg.norm(c_b_x_dottable) * np.linalg.norm(c_w_x_vector[1]))
        theta = acos(cos_theta)
        sign_cross = np.cross(c_b_x_dottable, c_w_x_vector[1])
        if sign_cross < 0:
            theta *= -1.
        return pose, theta

    def computeAllStates(self, tag_data):
        """
        Compute the Heading and Offset of the boat wrt the world coordinate system.
        Note: this function is done in the world coordinate system
        """
        all_states = {"boom": None,
                      "boat": None}
        if "ground" in tag_data.keys():
            # world wrt camera
            R_wc = tag_data["ground"][1]
            for (key, single_tag) in tag_data.items():
                if key != "ground":
                    #state = self.computeState(R_wc, single_tag)
                    state = self.computeStateV2(tag_data["ground"][0], single_tag[0])
                    all_states[key] = state
        return all_states;
            