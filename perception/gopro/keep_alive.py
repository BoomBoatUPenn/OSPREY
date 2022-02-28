from goprocam import GoProCamera
from goprocam import constants
import os
import sys

def connect_wifi():
    os.system("gopro-wifi")

def stream():
    gopro = GoProCamera.GoPro()
    gopro.stream("udp://127.0.0.1:10000")

if __name__ == "__main__":
    stream()