from unittest.mock import NonCallableMagicMock
import depthai as dai
import cv2
import numpy as np


class OAKPipeline():
    def __init__(self, stereo=False, april=False):
        self.__pipeline = dai.Pipeline()
        self.__width, self.__height = 1920, 1080
        self.__resize_factor = 2.0
        self.__fps = 60
        self.__stereo = stereo
        self.__april = april
        self.__streaming = False

        # rgb camera
        self.cam_rgb = self.__pipeline.create(dai.node.ColorCamera)
        self.cam_rgb.setFps(self.__fps)
        self.cam_rgb.setPreviewSize(int(self.__resize_factor * self.__width), int(self.__resize_factor * self.__height))
        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        self.xout_rgb = self.__pipeline.createXLinkOut()
        self.xout_rgb.setStreamName("rgb")
        self.cam_rgb.preview.link(self.xout_rgb.input)
        if stereo:
            # left camera
            self.cam_left = self.__pipeline.create(dai.node.MonoCamera)
            self.cam_left.setFps(self.__fps)
            self.cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            self.cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            self.xout_left = self.__pipeline.createXLinkOut()
            self.xout_left.setStreamName("left")
            # right camera
            self.cam_right = self.__pipeline.create(dai.node.MonoCamera)
            self.cam_right.setFps(self.__fps)
            self.cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            self.cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            self.xout_right = self.__pipeline.createXLinkOut()
            self.xout_right.setStreamName("right")
            # full stereo camera
            self.cam_stereo = self.__pipeline.create(dai.node.StereoDepth)
            self.cam_stereo.setLeftRightCheck(True)
            self.cam_stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            self.cam_stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
            self.xout_stereo = self.__pipeline.createXLinkOut()
            self.xout_stereo.setStreamName("depth")

            self.cam_left.out.link(self.cam_stereo.left)
            self.cam_right.out.link(self.cam_stereo.right)
            self.cam_stereo.disparity.link(self.xout_stereo.input)

        if april:
            self.aprilTag = self.__pipeline.create(dai.node.AprilTag)
            self.manip = self.__pipeline.create(dai.node.ImageManip)

            self.xout_AprilTag = self.__pipeline.create(dai.node.XLinkOut)
            self.xout_AprilTagImage = self.__pipeline.create(dai.node.XLinkOut)
            self.xout_AprilTag.setStreamName("aprilTagData")
            self.xout_AprilTagImage.setStreamName("aprilTagImage")
            
            self.manip.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
            self.aprilTag.initialConfig.setFamily(dai.AprilTagConfig.Family.TAG_36H11)
            # Linking
            self.aprilTag.passthroughInputImage.link(self.xout_AprilTagImage.input)
            self.cam_rgb.video.link(self.manip.inputImage)
            self.manip.out.link(self.aprilTag.inputImage)
            self.aprilTag.out.link(self.xout_AprilTag.input)
            self.aprilTag.inputImage.setBlocking(False)
            self.aprilTag.inputImage.setQueueSize(1)
            # configure parameters for apriltag
            aprilTagConfig = self.aprilTag.initialConfig.get()
            aprilTagConfig.quadDecimate = 4
            aprilTagConfig.quadSigma = 0
            aprilTagConfig.refineEdges = True
            aprilTagConfig.decodeSharpening = 0.25
            aprilTagConfig.maxHammingDistance = 1
            aprilTagConfig.quadThresholds.minClusterPixels = 5
            aprilTagConfig.quadThresholds.maxNmaxima = 15
            aprilTagConfig.quadThresholds.criticalDegree = 10
            aprilTagConfig.quadThresholds.maxLineFitMse = 10
            aprilTagConfig.quadThresholds.minWhiteBlackDiff = 5
            aprilTagConfig.quadThresholds.deglitch = False
            self.aprilTag.initialConfig.set(aprilTagConfig)

        return

    def startDevice(self):
        self.__device = dai.Device(self.__pipeline)
        self.__rgbQueue = self.__device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        if self.__stereo:
            self.__leftQueue = self.__device.getOutputQueue(name="left", maxSize=4, blocking=False)
            self.__rightQueue = self.__device.getOutputQueue(name="right", maxSize=4, blocking=False)
            self.__depthQueue = self.__device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        if self.__april:
            self.__manipQueue = self.__device.getOutputQueue(name="aprilTagImage", maxSize=8, blocking=False)
            self.__aprilQueue = self.__device.getOutputQueue(name="aprilTagData", maxSize=8, blocking=False)
        self.__device.startPipeline()
        self.__streaming = True
        return

    def read(self):
        frame_dict = {}
        rgb_frame = self.__rgbQueue.get()
        if rgb_frame is not None:
            rgb_im = rgb_frame.getCvFrame()
            frame_dict["rgb"] = rgb_im

        if self.__stereo:
            depth_frame = self.__depthQueue.get()
            if depth_frame is not None:
                depth_im = depth_frame.getFrame()
                depth_im_rgb = cv2.normalize(depth_im, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                depth_im_rgb = cv2.equalizeHist(depth_im_rgb)
                depth_im_rgb = cv2.applyColorMap(depth_im_rgb, cv2.COLORMAP_HOT)
            frame_dict["depth"] = depth_im_rgb
        
        if self.__april:
            aprilTagData = self.__aprilQueue.get().aprilTags
            tag_data = {}
            for aprilTag in aprilTagData:
                topLeft = aprilTag.topLeft
                topRight = aprilTag.topRight
                bottomRight = aprilTag.bottomRight
                bottomLeft = aprilTag.bottomLeft
                center = (int((topLeft.x + bottomRight.x) / 2), int((topLeft.y + bottomRight.y) / 2))
                points = center, (int(topLeft.x), int(topLeft.y)), (int(topRight.x), int(topRight.y)), (int(bottomRight.x), int(bottomRight.y)), (int(bottomLeft.x), int(bottomLeft.y)) 
                tag_data[str(aprilTag.id)] = points
            frame_dict["aprilTags"] = tag_data
        return frame_dict

    def isOpened(self):
        return self.__streaming


if __name__ == "__main__":
    stereo = False
    april = False
    oak_cam = OAKPipeline(stereo, april)
    oak_cam.startDevice()
    cv2.namedWindow("rgb")
    if stereo:
        cv2.namedWindow("depth")
    while oak_cam.isOpened():
        frame_dict = oak_cam.read()
        cv2.imshow("rgb", frame_dict["rgb"])
        if stereo:
            cv2.imshow("depth", frame_dict["depth"])
        if april:
            print(frame_dict["aprilTags"])
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
    cv2.destroyAllWindows()
