import cv2
import numpy as np
from time import time
import socket
from goprocam import GoProCamera
from goprocam import constants


def edgedetection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 10, 200)
    return edges;

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

def stream_cap(record):
    gpCam = GoProCamera.GoPro()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t=time()
    gpCam.livestream("start")
    gpCam.video_settings(res='720p', fps='12')
    gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.Default)
    gpCam.gpControlSet(constants.Stream.BIT_RATE, constants.Stream.BitRate.B2_4Mbps)
    cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)
    t=time()
    while True:
        nmat, frame = cap.read()
        canny = edgedetection(frame)
        frame = gridlines(frame)
        cv2.imshow("GoPro OpenCV", frame)
        cv2.imshow("GoPro OpenCV (Edges)", canny)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if time() - t >= 2.5:
            sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
            t=time()
    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()

record = False
stream_cap(record)
