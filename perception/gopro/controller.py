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
        self.__setpoint = 2.0 # x_distance from world tag
        self.controller = PID(self.__P, self.__I, self.__D, self.__setpoint)
        self.__high_speed = 1.0
        self.__low_speed = 1.0 / 3.0
        self.__mid_speed = 2.0 / 3.0
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
            elif alpha_mag < 20.0*(pi / 180.0):
                speed = self.__speed_scale*self.__mid_speed
            else:
                speed = self.__speed_scale*self.__low_speed
            self.speed = speed
            
    def command_boat(self, state):
        