# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 13:55:07 2018

@author: jack
"""
from PID import *
import subprocess
import time

def simSteps(experiment,timestep):

    print (experiment)
    if experiment == "Single":
        seconds = 6
    elif experiment == "Double":
        seconds = 20
    elif experiment == "Sergi":
        seconds = 40
    elif experiment == "Cube":
        seconds = 20

    return int(seconds/timestep)

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
