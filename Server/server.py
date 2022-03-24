from cv2 import threshold
from inputs import get_gamepad
import socket
import math
from threading import Thread
import time
from dataclasses import dataclass

@dataclass
class Boat:
    RudderNeutral: float
    RudderArc: float
    Rudder: float
    Throttle: float

UDP_IP_BOAT1 = "172.16.12.10"
UDP_IP_BOAT2 = "172.16.12.11"
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



MAX_THROTTLE = .25
Boat1 = Boat(.37, .5, .2, 0)
Boat2 = Boat(.37, .5, .2, 0)

def joystick():
    global x
    global y
    BTN_SOUTH_STATE = False
    BTN_NORTH_STATE = False
    BoatControlNumber = False
    Autonomous = False

    while 1:
        events = get_gamepad()
        for event in events:
            if (event.ev_type != "Sync"):
                if (event.code == 'ABS_RX'):
                    x = event.state
                elif (event.code == 'ABS_Y'):
                    y = event.state
                elif (event.code == 'BTN_NORTH'):
                    BTN_NORTH_STATE= not BTN_NORTH_STATE
                    if BTN_NORTH_STATE:
                        Autonomous = not Autonomous
                elif (event.code == 'BTN_SOUTH'):
                    BTN_SOUTH_STATE= not BTN_SOUTH_STATE
                    if BTN_SOUTH_STATE:
                        BoatControlNumber = not BoatControlNumber

                # circularize controller
                r = math.sqrt(x**2 + y**2)
                if not Autonomous:
                    if (r> radius):
                        y = math.floor(y * radius/r)
                        x = math.floor(x * radius/r)
                    
                    TargetBoat = Boat1 if not BoatControlNumber else Boat2
                    SecondBoat = Boat2 if not BoatControlNumber else Boat1

                    if y>0:
                        TargetBoat.Throttle = (y/radius) * MAX_THROTTLE
                    else:
                        TargetBoat.Throttle = 0
                    TargetBoat.Rudder = (x/radius)*TargetBoat.RudderArc + TargetBoat.RudderNeutral
                    SecondBoat.Throttle = 0
                    SecondBoat.Rudder = SecondBoat.RudderNeutral


def server():
    LastTime = [0,0,0]
    freq = [20, 5, 30]
        
    while 1:
        ms = time.time()*1000.0
        ## post at 20hz
        if (ms> LastTime[0] + 1000/freq[0]):
            LastTime[0] = ms
            CommandSocket.sendto(bytes('h', 'utf-8'), (UDP_IP_BOAT1, UDP_PORT_CMD))
            CommandSocket.sendto(bytes('h', 'utf-8'), (UDP_IP_BOAT2, UDP_PORT_CMD))
        if (ms> LastTime[1] + 1000/freq[1]):
            LastTime[1] = ms
            CommandSocket.sendto(bytes('t'+str(round(Boat1.Throttle, 2)), 'utf-8'), (UDP_IP_BOAT1, UDP_PORT_CMD))
            CommandSocket.sendto(bytes('r'+str(round(Boat1.Rudder, 2)), 'utf-8'), (UDP_IP_BOAT1, UDP_PORT_CMD))
            CommandSocket.sendto(bytes('t'+str(round(Boat2.Throttle, 2)), 'utf-8'), (UDP_IP_BOAT2, UDP_PORT_CMD))
            CommandSocket.sendto(bytes('r'+str(round(Boat2.Rudder, 2)), 'utf-8'), (UDP_IP_BOAT2, UDP_PORT_CMD))


def main():
    t1 = Thread(target = joystick)
    t1.start()
    server()
  
def launch():
    t1 = Thread(target = joystick)
    t2 = Thread(target= server)
    t1.start()
    t2.start()
    
if __name__=="__main__":
    main()

