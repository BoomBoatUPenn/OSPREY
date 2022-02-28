from inputs import get_gamepad
import socket
import math
from threading import Thread

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
from PyQt5.QtCore import Qt
import PyQt5
import sys, time
import re

UDP_IP = "172.16.12.10"
UDP_IP_BROADCAST = "172.16.12.255"
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

throttle = .00

MAX_THROTTLE = .25
rudder = 0.2
RudderNeutral = .37
RudderArc = .5


def joystick():
    global x
    global y
    global throttle
    global rudder
    while 1:
        events = get_gamepad()
        for event in events:
            if (event.ev_type != "Sync"):
                if (event.code == 'ABS_RX'):
                    x = event.state
                elif (event.code == 'ABS_Y'):
                    y = event.state

                # circularize controller
                r = math.sqrt(x**2 + y**2)
                
                if (r> radius):
                    y = math.floor(y * radius/r)
                    x = math.floor(x * radius/r)
                if y>0:
                    throttle = (y/radius) * MAX_THROTTLE
                else:
                    throttle = 0
                rudder = (x/radius)*RudderArc + RudderNeutral

def server():
    LastTime = [0,0,0]
    freq = [20, 5, 30]
    SentConfig = False
    global rudder
        
    while 1:
        if not SentConfig:
            CommandSocket.sendto(bytes('c' + str(8), 'utf-8'), (UDP_IP, UDP_PORT_CMD))
            SentConfig = True

        ms = time.time()*1000.0
        ## post at 20hz
        if (ms> LastTime[0] + 1000/freq[0]):
            LastTime[0] = ms
            CommandSocket.sendto(bytes('h', 'utf-8'), (UDP_IP, UDP_PORT_CMD))
        if (ms> LastTime[1] + 1000/freq[1]):
            LastTime[1] = ms
            CommandSocket.sendto(bytes('t'+str(round(throttle, 2)), 'utf-8'), (UDP_IP, UDP_PORT_CMD))
            CommandSocket.sendto(bytes('r'+str(round(rudder, 2)), 'utf-8'), (UDP_IP, UDP_PORT_CMD))




def main():
    t1 = Thread(target = joystick)
    t1.start()
    server()
  
if __name__=="__main__":
    main()

