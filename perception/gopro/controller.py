import numpy as np
from simple_pid import PID
from math import pi

class boat_pid():
    """
    Boat Rudder Angle PID Controller
    TODO:
        - Start with just proportional gain
        - Design controller to take in current heading angle, compute speed 
    """
    def __init__(self):
        self.__P = 1.0
        self.__I = 0.0
        self.__D = 0.0
        self.__setpoint = 2.0 # straight-line distance from origin along y-axis (meters)
        self.controller = PID(self.__P, self.__I, self.__D, self.__setpoint)
        self.__L = 1.0 # lookahead distance (meters)
        self.__high_speed = 1.0 # speed mapping
        self.__mid_speed = 2.0 / 3.0
        self.__low_speed = 1.0 / 3.0
        self.__speed_scale = 0.3
        self.speed = 0.0


    def set_P(self, new_P):
        self.P = new_P

    def set_I(self, new_I):
        self.I = new_I

    def set_D(self, new_D):
        self.D = new_D
    
    def set_speed(self, alpha):
        """
        Take in the current steering angle and compute speed (mapped 0-1) for the boat
        """
        alpha_mag = abs(alpha)
        if alpha_mag >= 0:
            if alpha_mag < 10.0*(pi / 180.0): # small steering angle -> scaled high speed
                speed = self.__speed_scale*self.__high_speed
            elif alpha_mag < 20.0*(pi / 180.0): # middle steering angle -> scaled mid speed
                speed = self.__speed_scale*self.__mid_speed
            else: # large steering angle -> scaled low speed
                speed = self.__speed_scale*self.__low_speed
            self.speed = speed
            return speed;
    
    def look_ahead(self, theta):
        """
        Use a lookahead distance to measure the position of the boat if it continued at current heading theta for distance L
        This will smooth out the controller and combat our latency issue a little bit.
        """
        lookahead = self.__L * cos(theta)
        return lookahead;

    def command_boat(self, state):
        """
        Use PID Controller to generate the steering angle of the boat
        Error function measures predicted signed distance of the boat from its setpoint
        """
        distance, theta = state
        distance_predicted = distance + self.look_ahead(theta)
        alpha = self.controller(distance_predicted) # call the controller to get a steering angle
        speed = self.set_speed(alpha)
        boat_command = (speed, alpha)
        return boat_command;

    # TODO:
    #  - Send Steering, Velocity Command to the Boat, maybe need to edit MCU
    #  - Simulate controller inputs and debug outputs
    #  - Accept inputs from perception module
    #  - Tune Gain params
    #  - Possibly rework according to justin's whiteboard talk