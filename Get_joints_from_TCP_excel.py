import os
import time
import math
import numpy as np

import pybullet as p
import pandas as pd
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict

from Rotations import *
from RobotDataBaseClass import RobotDataBase
from KinovaGen3Class import KinovaGen3

import pandas as pd

path_Excel = "/home/sergi/GitHub/Parameters_CrossEntrophy_TFM/test_tcp_59_averageconverted.xlsx"

p.connect(p.GUI)
p.setGravity(0.0, 0.0, -9.81)
robot = KinovaGen3()
robot.visual_inspection=False

df_Excel = pd.read_excel(path_Excel)
np_Excel = df_Excel.to_numpy()

np_positions = np_Excel[:,1:]


database = RobotDataBase()

for i in range(np_positions.shape[0]):
    position = np_positions[i,:3]
    quaternion = np_positions[i,3:]
    robot.move_cartesian([position,quaternion],wait=True,max_iterations = 10**5)
    joints = robot.get_actual_control_joints_angle()
    database.joints_angles_rad.append(joints)

dataframe = pd.DataFrame({})

joints_array = np.array(database.joints_angles_rad)

dataframe["joint 0"] = joints_array[:,0]
dataframe["joint 1"] = joints_array[:,1]
dataframe["joint 2"] = joints_array[:,2]

dataframe["joint 3"] = joints_array[:,3]
dataframe["joint 4"] = joints_array[:,4]
dataframe["joint 5"] = joints_array[:,5]

dataframe.index = np_Excel[:,0]

dataframe.to_excel("joints_from_positions_59.xlsx")
