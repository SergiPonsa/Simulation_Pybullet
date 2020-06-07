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

path_Excel = "/home/sergi/GitHubRishabh/gen3-mujoco_mod/Joint_Trajectori_19converted.xlsx"

p.connect(p.GUI)
p.setGravity(0.0, 0.0, -9.81)
robot = KinovaGen3()
robot.visual_inspection=False

df_Excel = pd.read_excel(path_Excel)
np_Excel = df_Excel.to_numpy()

np_joints = np_Excel[:,1:]

database = RobotDataBase()
for i in range(np_joints.shape[0]):
    joints_target = list(np_joints[i,:])
    robot.move_joints(joint_param_value=joints_target,wait=True)
    robot.get_actual_tcp_pose()
    [tcp_position, tcp_orientation_q] = robot.get_actual_tcp_pose()
    database.tcp_position.append(tcp_position)
    database.tcp_orientation_q.append(tcp_orientation_q)
    database.tcp_orientation_e.append(p.getEulerFromQuaternion(tcp_orientation_q))

dataframe = pd.DataFrame({})

tcp_position_array = np.array(database.tcp_position)
tcp_orientation_e_array = np.array(database.tcp_orientation_e)

dataframe["pos x"] = tcp_position_array[:,0]
dataframe["pos y"] = tcp_position_array[:,1]
dataframe["pos z"] = tcp_position_array[:,2]

dataframe["ori x"] = tcp_orientation_e_array[:,0]
dataframe["ori y"] = tcp_orientation_e_array[:,1]
dataframe["ori z"] = tcp_orientation_e_array[:,2]

dataframe.index = np_Excel[:,0]

dataframe.to_excel("positions_from_joints.xlsx")
