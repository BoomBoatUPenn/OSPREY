from inputs import get_gamepad
import socket
import math
from threading import Thread

from controller import boat_pid

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
from PyQt5.QtCore import Qt
import PyQt5
import sys, time
import re


# globals to be updated in pid thread
speed_pid_output = 0
alpha_pid_output = 0


UDP_IP = "192.168.1.6"
UDP_IP_BROADCAST = "192.168.1.255"
UDP_PORT_CMD = 5005
UDP_PORT_TELEM = 5006
radius= 32768

CommandSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
TelemSocket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
TelemSocket.bind(('',UDP_PORT_TELEM))
TelemSocket.setblocking(0)

x = 0
y = 0

speed = .01

def joystick():
    BTN_TR_state = False
    BTN_SOUTH_STATE = False
    BTN_WEST_STATE = False
    BTN_WEST_EAST = False
    global x
    global y
    global Grip
    global WallFollow
    global MoveTo
    global BeaconTrack
    while 1:
        events = get_gamepad()
        for event in events:
            if (event.ev_type != "Sync"):
                if (event.code == 'ABS_X'):
                    x = event.state
                elif (event.code == 'ABS_Y'):
                    y = event.state
                elif (event.code == 'BTN_TR'):
                    BTN_TR_state= not BTN_TR_state
                    if BTN_TR_state:
                        Grip = not Grip
                elif (event.code == 'BTN_SOUTH'):
                    BTN_SOUTH_STATE= not BTN_SOUTH_STATE
                    if BTN_SOUTH_STATE:
                        WallFollow = not WallFollow
                elif (event.code == 'BTN_WEST'):
                    BTN_WEST_STATE= not BTN_WEST_STATE
                    if BTN_WEST_STATE:
                        MoveTo = not MoveTo
                elif (event.code == 'BTN_EAST'):
                    BTN_WEST_EAST= not BTN_WEST_EAST
                    if BTN_WEST_EAST:
                        BeaconTrack = not BeaconTrack

                # circularize controller
                r = math.sqrt(x**2 + y**2)
                
                if (r> radius):
                    y = math.floor(y * radius/r)
                    x = math.floor(x * radius/r)

def server():
    LastTime = [0,0,0]
    freq = [20, 5, 3]
    SentConfig = False
        
    while 1:
        if not SentConfig:
            CommandSocket.sendto(bytes('c' + str(8), 'utf-8'), (UDP_IP, UDP_PORT_CMD))
            SentConfig = True

        ms = time.time()*1000.0
        ## post at 20hz
        if (ms> LastTime[0] + 1000/freq[0]):
            LastTime[0]=ms
            CommandSocket.sendto(bytes('s'+str(round(speed,2)), 'utf-8'), (UDP_IP, UDP_PORT_CMD))


def get_boat_state():
    return 0, 0


def run_controller(recalculate_interval_ms=100):
    b = boat_pid()
    while True:
        distance, theta = get_boat_state()  # from perception
        speed_pid_output, alpha_pid_output = b.command_boat((distance, theta))
        time.sleep(recalculate_interval_ms * 0.001)


def main():
    t1 = Thread(target=run_controller, args=(100,))
    t1.start()
    server()
  
if __name__=="__main__":
    main()

