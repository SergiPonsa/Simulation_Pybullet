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
from RobotClassDev import Robot
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict



# ------------------------------------------------------------------------------------------------------

class KinovaGen3(Robot):
    """Kinova-Gen3 Robot class"""

    #Define the parameters of Robot
    def __init__(self,
    urdf_root = pybullet_data.getDataPath(),
    root = "",
    robot_urdf = "models/urdf/JACO3_URDF_V11.urdf",

    robot_launch_pos = [0, 0, 0],
    robot_launch_orien = p.getQuaternionFromEuler([0, 0, 0]),

    last_robot_joint_name = "EndEffector",
    robot_control_joints = ["Actuator1","Actuator2","Actuator3","Actuator4","Actuator5","Actuator6","Actuator7"],
    robot_mimic_joints_name = [],
    robot_mimic_joints_master = [],
    robot_mimic_multiplier = [],
    tool_orient_e = [-3.14,0,1.57],

    nullspace = True,
    home_angles = [0, 0.785, 0.0, 0.785, 0.0, 1.57, 0.0],
    visual_inspection = True,

    tcp_offset_pos = [0.0, 0.0, 0.0],
    tcp_offset_orien_e = [0.0, 0.0, 0.0]):

        #Give the parameters to the robot class
        super(KinovaGen3, self).__init__(urdf_root = urdf_root,
        root = root,
        robot_urdf = robot_urdf,

        robot_launch_pos = robot_launch_pos,
        robot_launch_orien = robot_launch_orien,

        last_robot_joint_name = last_robot_joint_name,
        robot_control_joints = robot_control_joints,
        robot_mimic_joints_name = robot_mimic_joints_name,
        robot_mimic_joints_master = robot_mimic_joints_master,
        robot_mimic_multiplier = robot_mimic_multiplier,
        tool_orient_e = tool_orient_e,

        nullspace = nullspace,
        home_angles = home_angles,
        visual_inspection = visual_inspection,

        tcp_offset_pos = tcp_offset_pos,
        tcp_offset_orien_e = tcp_offset_orien_e)

# ------------------------------------------------------------------------------------------------------

if __name__ == '__main__':

    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)



    # create robot instance with default configuration
    robot = KinovaGen3()
    print("Class created")
    robot.move_home()

    #Modify all the links mass , don't specify the name

    file_2_write = robot.create_file_to_modify(robot.robot_urdf)
    print(file_2_write)
    if (file_2_write != "error"):
        robot.modify_urdf(robot.robot_urdf,file_2_write,"mass",2.0)

    #Modify only one link mass , specify the name

    info = p.getJointInfo(robot.robot_id,robot.last_robot_joint_index)
    LastLinkName = str(info[12], "utf-8")
    print(LastLinkName)

    file_2_write = robot.create_file_to_modify(robot.robot_urdf)
    print(file_2_write)
    if (file_2_write != "error"):
        robot.modify_urdf(robot.robot_urdf,file_2_write,"mass",2.0,\
        link_or_joint_name=LastLinkName)
    print("created")
    """
    Test of functions

    joint_angle_target = list(robot.home_angles)
    time.sleep(2)
    joint_angle_target[5] -= 1.57
    robot.move_joints(joint_param_value=joint_angle_target)
    time.sleep(2)
    joint_angle_target[5] -= 1.57
    robot.move_joints(joint_param_value=joint_angle_target)
    print("last move")
    [actual_position,actual_orientation_q] = list(robot.get_actual_tcp_pose())
    print(actual_position + p.getEulerFromQuaternion(actual_orientation_q))

    robot.move_home()
    print("move home")

    [actual_position,actual_orientation_q] = list(robot.get_actual_tcp_pose())
    print(actual_position + p.getEulerFromQuaternion(actual_orientation_q))

    desired_position = [0.6,0.0,0.3]
    desired_orientation_q = p.getQuaternionFromEuler([0.0,0.0,0.0])
    pose_to_go = list(robot.tcp_go_pose(desired_position,desired_orientation_q))
    robot.move_cartesian(pose_to_go)
    print("position 1")
    [actual_position,actual_orientation_q] = list(robot.get_actual_tcp_pose())
    print(actual_position + p.getEulerFromQuaternion(actual_orientation_q))
    time.sleep(10)

    desired_position [0] -= 0.1

    pose_to_go = list(robot.tcp_go_pose(desired_position,desired_orientation_q))
    robot.move_cartesian(pose_to_go)
    print("position 2")
    [actual_position,actual_orientation_q] = list(robot.get_actual_tcp_pose())
    print(actual_position + p.getEulerFromQuaternion(actual_orientation_q))
    time.sleep(10)


    desired_position [1] += 0.1

    pose_to_go = list(robot.tcp_go_pose(desired_position,desired_orientation_q))
    robot.move_cartesian(pose_to_go)
    print("position 3")
    [actual_position,actual_orientation_q] = list(robot.get_actual_tcp_pose())
    print(actual_position + p.getEulerFromQuaternion(actual_orientation_q))
    time.sleep(10)



    desired_orientation_q = p.getQuaternionFromEuler([0.0,0.0,1.57])
    pose_to_go = list(robot.tcp_go_pose(desired_position,desired_orientation_q))
    print("position 4")
    robot.move_cartesian(pose_to_go)
    [actual_position,actual_orientation_q] = list(robot.get_actual_tcp_pose())
    print(actual_position + p.getEulerFromQuaternion(actual_orientation_q))
    time.sleep(10)

    desired_orientation_q = p.getQuaternionFromEuler([0.0,0.0,-1.57])
    pose_to_go = list(robot.tcp_go_pose(desired_position,desired_orientation_q))
    print("position 5")
    robot.move_cartesian(pose_to_go)
    [actual_position,actual_orientation_q] = list(robot.get_actual_tcp_pose())
    print(actual_position + p.getEulerFromQuaternion(actual_orientation_q))
    time.sleep(10)

    desired_orientation_q = p.getQuaternionFromEuler([0.0,-0.785,-1.57])
    pose_to_go = list(robot.tcp_go_pose(desired_position,desired_orientation_q))
    print("position 6")
    robot.move_cartesian(pose_to_go)
    [actual_position,actual_orientation_q] = list(robot.get_actual_tcp_pose())
    print(actual_position + p.getEulerFromQuaternion(actual_orientation_q))
    time.sleep(10)
    """




    try:
        while True:
            robot.step_simulation()
    except KeyboardInterrupt:
        p.disconnect()

# ------------------------------------------------------------------------------------------------------
