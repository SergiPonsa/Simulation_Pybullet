# -*- coding: utf-8 -*-
"""
Created on Thu Aug  9 16:39:38 2018

@author: jack
"""
#import csv

class PID(object):
    def __init__(self,max_velocity = 30,kp=0.1,ki=0.0,kd=0.0):
        self.max_velocity = max_velocity

        self._kp = kp
        self._ki = ki
        self._kd = kd


        self._target_theta = 0.0
        self._sampling_time = 0.01

        self._theta0 = 0.0
        self._thetai = 0.0

    def init_status(self):
        self._theta0 = 0.0
        self._thetai = 0.0

    def set_target_theta(self, theta, degrees = True):
        if(degrees==True):
            self._target_theta = theta
        else:
            self._target_theta = theta*(180/3.14)

    def get_target_theta(self):
        return self._target_theta

    def get_velocity(self, theta , max_velocity = None):
        if(max_velocity == None):
            max_velocity = self.max_velocity

        error = self._target_theta - theta
        self._thetai += error * self._sampling_time
        dtheta = (error - self._theta0) / self._sampling_time
        self._theta0 = error


        duty_ratio = (error * self._kp + self._thetai * self._ki + dtheta * self._kd)/self._sampling_time

        if duty_ratio > max_velocity:
            duty_ratio = max_velocity
        elif duty_ratio < -max_velocity:
            duty_ratio = -max_velocity

        return duty_ratio
