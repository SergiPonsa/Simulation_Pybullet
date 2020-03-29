# name: Kinova-Gen3-Class.py
# content: Kinova Gen3 robot children of Robot class
# # author: BitMetrics (Sergi Ponsà)
# date: 03-2020
# ------------------------------------------------------------------------------------------------------

import os
import time
import pdb
import math
import numpy as np
import pybullet as p
import RobotClass-Dev
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict
from constants import TCP_INIT_POSE



# ------------------------------------------------------------------------------------------------------

class Kinova-Gen3(Robot):
    """Kinova-Gen3 Robot class"""
    def __init__(self,
    urdf_root = pybullet_data.getDataPath(),
    root = "",
    robot_urdf = "/models/urdf/JACO3_URDF_V11.urdf",

    robot_launch_pos = [0, 0, 0],
    robot_launch_orien = p.getQuaternionFromEuler([0, 0, 0]),

    last_robot_joint_name = "",
    robot_control_joints = [],
    robot_mimic_joints_name = [],
    robot_mimic_joints_master = [],
    robot_mimic_multiplier = [],

    nullspace = True,
    home_angles = [-0.992, -1.157, 1.323, -1.720, -1.587, 0.0],
    visual_inspection = True,

    tcp_offset_pos = [0.0, 0.0, 0.0],
    tcp_offset_orien = [0.0, 0.0, 0.0]):

        super(Kinova-Gen3, self).__init__()

# ------------------------------------------------------------------------------------------------------

if __name__ == '__main__':

    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    # create robot instance with default configuration
    robot = Robot()
    robot.move_home()
    try:
        while True:
            robot.step_simulation()
    except KeyboardInterrupt:
        p.disconnect()

# ------------------------------------------------------------------------------------------------------
