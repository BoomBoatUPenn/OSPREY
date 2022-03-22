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

from controller import boat_pid	

# globals to be updated in pid thread	
pid_mode = False	
speed_pid_output = 0	
alpha_pid_output = 0


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


def scale_rudder(rudder_angle):
    return ((RudderArc / 2) * rudder_angle) + RudderNeutral


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
                    print("Rudder: ", x)
                elif (event.code == 'ABS_Y'):
                    y = event.state
                    print("Throttle", y)
                # circularize controller
                r = math.sqrt(x**2 + y**2)
                
                if (r> radius):
                    y = math.floor(y * radius/r)
                    x = math.floor(x * radius/r)
                if y>0:
                    throttle = (y/radius) * MAX_THROTTLE
                else:
                    throttle = .00
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



	# need to get from perception module	
def get_boat_state():	
    return 0, 0	

def run_perception():	
    pass	

def run_controller(recalculate_interval_ms=100):	
    b = boat_pid(MAX_THROTTLE)	
    while True:	
        distance, theta = get_boat_state()  # from perception	
        speed_pid_output, alpha_pid_output = b.command_boat((distance, theta))	
        time.sleep(recalculate_interval_ms * 0.001)


def main():
    t1 = Thread(target = joystick)
    t1.start()

    if pid_mode:	
        t2 = Thread(target=run_controller, args=(100,))	
        t3 = Thread(target=run_perception)	
        t2.start()	
        t3.start()	
        
    server()
  
if __name__=="__main__":
    main()


"""
Notes:

Things Eliminated from possibility:
    - Xbox remote not connecting to server (can print commands, they change with joystick input)
    - ESP32 not connecting to server (moving the joystick triggers changes in values for throttle and rudder angle)
    - Battery to ESC solder points (removed electrical tape to check)
    - ESC to motor solder points (removed electrical tape to check)

Not sure about:
    - ESP32 pins not connecting to motor (maybe the solder leads on the printed board? i trust zach's work though)
    - ESC Calibration (tried manually inputting throttle commands with no luck, but not sure if the range i tried even makes sense)
"""