# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 13:55:07 2018

@author: jack
"""
from PID import *
import subprocess
import time

def simSteps(experiment,timestep):

    #print (experiment)
    if experiment == "Single":
        seconds = 6
    elif experiment == "Double":
        seconds = 20
    elif experiment == "Sergi":
        seconds = 40
    elif experiment == "Cube":
        seconds = 20
    elif experiment == "Training1":
        seconds = 30
    simSteps_value = (seconds/timestep)
    return int(simSteps_value)

def set_target_thetas(num_steps, pid, experiment,simulator, simStep , joint = 1):
    if experiment == "Single":
        if simStep == 0:
            pid[joint].set_target_theta(-90)
        else:
            return pid

    elif experiment == "Double":
        if simStep == 0:
            pid[1].set_target_theta(-90)
            pid[4].set_target_theta(-90)
        elif simStep % (num_steps*0.5) == 0:
            pid[1].set_target_theta(-90)
            pid[4].set_target_theta(90)
        elif simStep % (num_steps*0.25) == 0:
            pid[1].set_target_theta(0)
            pid[4].set_target_theta(0)
        else:
            return pid

    elif experiment == "Sergi":
        if simStep == 0:
            pid[1].set_target_theta(-90)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(-90)
        elif simStep % (num_steps*0.875) == 0:
            pid[1].set_target_theta(0)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(90)
        elif simStep % (num_steps*0.75) == 0:
            pid[1].set_target_theta(0)
            pid[3].set_target_theta(90)
            pid[4].set_target_theta(90)
        elif simStep % (num_steps*0.625) == 0:
            pid[1].set_target_theta(0)
            pid[3].set_target_theta(90)
            pid[4].set_target_theta(0)
        elif simStep % (num_steps*0.5) == 0:
            pid[1].set_target_theta(0)
            pid[3].set_target_theta(90)
            pid[4].set_target_theta(-90)
        elif simStep % (num_steps*0.375) == 0:
            pid[1].set_target_theta(0)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(-90)
        elif simStep % (num_steps*0.25) == 0:
            pid[1].set_target_theta(0)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(-45)
        elif simStep % (num_steps*0.125) == 0:
            pid[1].set_target_theta(0)
            pid[3].set_target_theta(0)
            pid[4].set_target_theta(0)
        else:
            return pid
    elif experiment == "Training1":

        if simStep == 0:
            #Move home
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(22.46)
            pid[2].set_target_theta(0)
            pid[3].set_target_theta(112.41)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(44.69)
            pid[6].set_target_theta(0)

        elif simStep % (num_steps*0.875) == 0:
            #Negative Z
            pid[0].set_target_theta(1.88)
            pid[1].set_target_theta(44.61)
            pid[2].set_target_theta(-0.25)
            pid[3].set_target_theta(118.98)
            pid[4].set_target_theta(0.59)
            pid[5].set_target_theta(16.18)
            pid[6].set_target_theta(1.14)
        elif simStep % (num_steps*0.75) == 0:
            #Positive Z
            pid[0].set_target_theta(0.67)
            pid[1].set_target_theta(9.63)
            pid[2].set_target_theta(1.28)
            pid[3].set_target_theta(83.49)
            pid[4].set_target_theta(-0.229)
            pid[5].set_target_theta(86.49)
            pid[6].set_target_theta(1.99)
        elif simStep % (num_steps*0.625) == 0:
            #Negative Y
            pid[0].set_target_theta(26.29)
            pid[1].set_target_theta(26.89)
            pid[2].set_target_theta(3.14)
            pid[3].set_target_theta(97.61)
            pid[4].set_target_theta(-1.98)
            pid[5].set_target_theta(55.26)
            pid[6].set_target_theta(30.14)
        elif simStep % (num_steps*0.5) == 0:
            #Positive Y
            pid[0].set_target_theta(-24.44)
            pid[1].set_target_theta(23.11)
            pid[2].set_target_theta(-2.88)
            pid[3].set_target_theta(103.86)
            pid[4].set_target_theta(1.67)
            pid[5].set_target_theta(52.77)
            pid[6].set_target_theta(-28.05)
        elif simStep % (num_steps*0.375) == 0:
            #Move home
            pid[0].set_target_theta(0)
            pid[1].set_target_theta(22.46)
            pid[2].set_target_theta(0)
            pid[3].set_target_theta(112.41)
            pid[4].set_target_theta(0)
            pid[5].set_target_theta(44.69)
            pid[6].set_target_theta(0)
        elif simStep % (num_steps*0.25) == 0:
            # Negative x
            pid[0].set_target_theta(2.74)
            pid[1].set_target_theta(-5.21)
            pid[2].set_target_theta(0.71)
            pid[3].set_target_theta(139.87)
            pid[4].set_target_theta(0.05)
            pid[5].set_target_theta(44.93)
            pid[6].set_target_theta(3.41)
        elif simStep % (num_steps*0.125) == 0:
            # Positive x
            pid[0].set_target_theta(0.68)
            pid[1].set_target_theta(43.38)
            pid[2].set_target_theta(0.87)
            pid[3].set_target_theta(66.77)
            pid[4].set_target_theta(-0.65)
            pid[5].set_target_theta(69.50)
            pid[6].set_target_theta(1.54)






        else:
            return pid


    elif experiment == "Cube":
        if simStep == 0:
            pid[1].set_target_theta(30)
            pid[2].set_target_theta(120)
            pid[3].set_target_theta(90)
        elif  num_steps*0.4 == simStep:
            pid[1].set_target_theta(-60)
            pid[2].set_target_theta(65)
            pid[3].set_target_theta(90)
        elif num_steps*0.7 == simStep:
            pid[1].set_target_theta(-67)
            pid[2].set_target_theta(30)
            pid[3].set_target_theta(90)
        else:
            return pid
    #if simulator == "PyBullet":
        #pid = pid[-2:] + pid[0:-2]

    return pid
