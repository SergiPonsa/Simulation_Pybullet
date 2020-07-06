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

#create enviorment
p.connect(p.GUI)
p.setGravity(0.0, 0.0, -9.81)


#import robot
robot = KinovaGen3()
robot.move_home()

#create cartesian control
root = ""
catersian_urdf = "models/urdf/Cartesian_control.urdf"
cartesian_control_id = p.loadURDF(os.path.join(root, catersian_urdf),[0.1,0,0],p.getQuaternionFromEuler([0, 0, 0]),flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT )
connect_cube = p.addUserDebugParameter("connect cube to robot",1,0,0)
send_data = p.addUserDebugParameter("send data button",1,0,0)
inc_x = p.addUserDebugParameter("increment x",-1,1,0)
inc_y = p.addUserDebugParameter("increment y",-1,1,0)
inc_z = p.addUserDebugParameter("increment z",-1,1,0)
p.stepSimulation()
time.sleep(1./240.)
printhi=0
connect_cube_b = 0
send_data_b = 0
old_connect_cube_b = 0
old_send_data_b = 0
distance_by_action = 0.05

while True:
    connect_cube_b = p.readUserDebugParameter(connect_cube)
    send_data_b = p.readUserDebugParameter(send_data)
    keys = p.getKeyboardEvents()
    print(keys)
    # p.stepSimulation()
    # time.sleep(1./240.)

    if (printhi == 0):
        print ("Hi")
        print(cartesian_control_id)
        for i in range(4):
            info = p.getJointInfo(cartesian_control_id, i)
            print("info",info,"\n")

        print("connect_cube_b",connect_cube_b)
        print("send_data_b",send_data_b)
        printhi=1

    if (connect_cube_b != old_connect_cube_b):
        old_connect_cube_b = connect_cube_b
        print("connect_cube_b",connect_cube_b)

        p.createConstraint(robot.robot_id,robot.last_robot_joint_index,\
                            cartesian_control_id, 3, p.JOINT_POINT2POINT,[0, 0, 0], [0, 0, 0], [0, 0, 0],\
                            childFrameOrientation = p.getQuaternionFromEuler([0,0,0]))

        p.createConstraint(robot.robot_id,robot.last_robot_joint_index-2,\
                            cartesian_control_id, 4, p.JOINT_POINT2POINT,[0, 0, 0], [0, 0, 0], [0, 0, 0],\
                            childFrameOrientation = p.getQuaternionFromEuler([0,0,0]))

        for i in range(240):
            p.stepSimulation()
        robot.move_joints(joint_param_value = robot.home_angles, desired_force_per_one_list = [0.1],wait=False)
        #p.createConstraint(robot.robot_id,robot.last_robot_joint_index,\
                            #cartesian_control_id, 3, p.JOINT_POINT2POINT,[0, 0, 0], [0, 0, 0], [0, 0, 0])

    if (send_data_b != old_send_data_b):
        old_send_data_b = send_data_b
        print("send_data_b",send_data_b)

        cart_pos = [0,0,0]
        cart_pos[0] = p.getJointState(cartesian_control_id,3)[0]
        cart_pos[1] = p.getJointState(cartesian_control_id,2)[0]
        cart_pos[2] = p.getJointState(cartesian_control_id,1)[0]

        print("initial cartesian position",cart_pos,"\n")

        inc_x_v = p.readUserDebugParameter(inc_x)*distance_by_action + cart_pos[0]
        inc_y_v = p.readUserDebugParameter(inc_y)*distance_by_action + cart_pos[1]
        inc_z_v = p.readUserDebugParameter(inc_z)*distance_by_action + cart_pos[2]
        print("inc_x_v",inc_x_v)
        print("inc_y_v",inc_y_v)
        print("inc_z_v",inc_z_v)

        p.setJointMotorControl2(cartesian_control_id, 1,\
                                p.POSITION_CONTROL, targetPosition = inc_z_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)
        p.setJointMotorControl2(cartesian_control_id, 2,\
                                p.POSITION_CONTROL, targetPosition = inc_y_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)
        p.setJointMotorControl2(cartesian_control_id, 3,\
                                p.POSITION_CONTROL, targetPosition = inc_x_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)

        print("simulate")
        p.stepSimulation()
        time.sleep(1./240.)

        for i in range(1,4):
            print(p.getJointState(cartesian_control_id,i)[0])

    qKey = ord('o')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        cart_pos = [0,0,0]
        cart_pos[2] = p.getJointState(cartesian_control_id,1)[0]
        inc_z_v = 1*distance_by_action + cart_pos[2]

        p.setJointMotorControl2(cartesian_control_id, 1,\
                                p.POSITION_CONTROL, targetPosition = inc_z_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)
        p.stepSimulation()
        time.sleep(1./240.)

    qKey = ord('u')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        cart_pos = [0,0,0]
        cart_pos[2] = p.getJointState(cartesian_control_id,1)[0]
        inc_z_v = -1*distance_by_action + cart_pos[2]
        p.setJointMotorControl2(cartesian_control_id, 1,\
                                p.POSITION_CONTROL, targetPosition = inc_z_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)
        p.stepSimulation()
        time.sleep(1./240.)

    qKey = ord('l')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        cart_pos = [0,0,0]
        cart_pos[1] = p.getJointState(cartesian_control_id,2)[0]
        inc_y_v = 1*distance_by_action + cart_pos[1]

        p.setJointMotorControl2(cartesian_control_id, 2,\
                                p.POSITION_CONTROL, targetPosition = inc_y_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)
        p.stepSimulation()
        time.sleep(1./240.)

    qKey = ord('j')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        cart_pos = [0,0,0]
        cart_pos[1] = p.getJointState(cartesian_control_id,2)[0]
        inc_y_v = -1*distance_by_action + cart_pos[1]
        p.setJointMotorControl2(cartesian_control_id, 2,\
                                p.POSITION_CONTROL, targetPosition = inc_y_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)
        p.stepSimulation()
        time.sleep(1./240.)

    qKey = ord('i')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        cart_pos = [0,0,0]
        cart_pos[0] = p.getJointState(cartesian_control_id,3)[0]
        inc_x_v = 1*distance_by_action + cart_pos[0]

        p.setJointMotorControl2(cartesian_control_id, 3,\
                                p.POSITION_CONTROL, targetPosition = inc_x_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)
        p.stepSimulation()
        time.sleep(1./240.)

    qKey = ord('k')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        cart_pos = [0,0,0]
        cart_pos[0] = p.getJointState(cartesian_control_id,3)[0]
        inc_x_v = -1*distance_by_action + cart_pos[0]
        p.setJointMotorControl2(cartesian_control_id, 3,\
                                p.POSITION_CONTROL, targetPosition = inc_x_v,force = 10000.0,maxVelocity=2.0,positionGain=1.5,\
                                    velocityGain=1.1)
        p.stepSimulation()
        time.sleep(1./240.)
