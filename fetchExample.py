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



def moveCartesian(cartesian_id,xpos = 0.0, ypos = 0.0, zpos = 0.0,force = 1000.0, vel = 3.0 ):
    p.setJointMotorControl2(cartesian_id, 1,\
                            p.POSITION_CONTROL, targetPosition = zpos,force = force,maxVelocity = vel,positionGain=1.5,\
                                velocityGain=1.1)
    p.setJointMotorControl2(cartesian_id, 2,\
                            p.POSITION_CONTROL, targetPosition = ypos,force = force,maxVelocity = vel,positionGain=1.5,\
                                velocityGain=1.1)
    p.setJointMotorControl2(cartesian_id, 3,\
                            p.POSITION_CONTROL, targetPosition = xpos,force = force,maxVelocity = vel,positionGain=1.5,\
                                velocityGain=1.1)


def moveCartesianOffset(cartesian_id,inc_x = 0.0, inc_y = 0.0, inc_z = 0.0,force = 1000.0, vel = 1.0 ):

    cart_pos = [0]*3
    cart_pos[0] = p.getJointState(cartesian_control_id,3)[0]
    cart_pos[1] = p.getJointState(cartesian_control_id,2)[0]
    cart_pos[2] = p.getJointState(cartesian_control_id,1)[0]

    xpos = inc_x*distance_by_action + cart_pos[0]
    ypos = inc_y*distance_by_action + cart_pos[1]
    zpos = inc_z*distance_by_action + cart_pos[2]

    moveCartesian(cartesian_id,xpos = xpos, ypos = ypos, zpos = zpos,force = force, vel = vel )

#create enviorment
p.connect(p.GUI)
p.setGravity(0.0, 0.0, -9.81)


#import robot
robot = KinovaGen3(time_step=0.002)
repeats = 20
robot.move_home()

#create cartesian control
root = ""
catersian_urdf = "models/urdf/Cartesian_control.urdf"
cartesian_control_id = p.loadURDF(os.path.join(root, catersian_urdf),[0.1,0,0],p.getQuaternionFromEuler([0, 0, 0]),flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT )

#create flat ground
bullet_path = pybullet_data.getDataPath()
plane_id=p.loadURDF(os.path.join(bullet_path, "plane.urdf"), [0, 0, -0.01])

nExp = "101"
#create puck
puck_urdf = "models/urdf/Puck.urdf"
puck_id=p.loadURDF(os.path.join(root, puck_urdf), [0.5, -0.2, 0.05])

#create goal
goal_urdf = "models/urdf/Goal.urdf"
goal_id=p.loadURDF(os.path.join(root, goal_urdf), [0.6, -0.4, 0.0])

connect_cube = p.addUserDebugParameter("connect cube to robot",1,0,0)
send_data = p.addUserDebugParameter("send data button",1,0,0)
inc_x = p.addUserDebugParameter("increment x",-1,1,0)
inc_y = p.addUserDebugParameter("increment y",-1,1,0)
inc_z = p.addUserDebugParameter("increment z",-1,1,0)
p.stepSimulation()
time.sleep(1./240.)

#Connect elements
cons1=p.createConstraint(robot.robot_id,robot.last_robot_joint_index,\
                    cartesian_control_id, 3, p.JOINT_POINT2POINT,[0, 0, 0], [0, 0, 0], [0, 0, 0],\
                    childFrameOrientation = p.getQuaternionFromEuler([0,0,0]))

cons2=p.createConstraint(robot.robot_id,robot.last_robot_joint_index-2,\
                    cartesian_control_id, 4, p.JOINT_POINT2POINT,[0, 0, 0], [0, 0, 0], [0, 0, 0],\
                    childFrameOrientation = p.getQuaternionFromEuler([0,0,0]))
p.changeConstraint(cons1,maxForce=1000)
p.changeConstraint(cons2,maxForce=1000)
for i in range(240):
    p.stepSimulation()
robot.move_joints(joint_param_value = robot.home_angles, desired_force_per_one_list = [0.5,0.5,0.5,0.5,0.5,0.5,2],wait=False)

robot.database_name = "new"
#Move Robot to the desired position

moveCartesian(cartesian_control_id,xpos = 0.036, ypos = 0.0, zpos = 0.55)
for i in range(240):
    p.stepSimulation()

printhi = 0
connect_cube_b = 0
send_data_b = 0
old_connect_cube_b = 0
old_send_data_b = 0
distance_by_action = 0.05
inc_value=0.25
actions = []
pos_puck = []
pos_goal = []
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

        cons1=p.createConstraint(robot.robot_id,robot.last_robot_joint_index,\
                            cartesian_control_id, 3, p.JOINT_POINT2POINT,[0, 0, 0], [0, 0, 0], [0, 0, 0],\
                            childFrameOrientation = p.getQuaternionFromEuler([0,0,0]))

        cons2=p.createConstraint(robot.robot_id,robot.last_robot_joint_index-2,\
                            cartesian_control_id, 4, p.JOINT_POINT2POINT,[0, 0, 0], [0, 0, 0], [0, 0, 0],\
                            childFrameOrientation = p.getQuaternionFromEuler([0,0,0]))
        p.changeConstraint(cons1,maxForce=100)
        p.changeConstraint(cons2,maxForce=100)
        for i in range(240):
            p.stepSimulation()
        robot.move_joints(joint_param_value = robot.home_angles, desired_force_per_one_list = [0.2],wait=False)
        #p.createConstraint(robot.robot_id,robot.last_robot_joint_index,\
                            #cartesian_control_id, 3, p.JOINT_POINT2POINT,[0, 0, 0], [0, 0, 0], [0, 0, 0])

    if (send_data_b != old_send_data_b):
        old_send_data_b = send_data_b
        print("send_data_b",send_data_b)

        inc_x_v = p.readUserDebugParameter(inc_x)
        inc_y_v = p.readUserDebugParameter(inc_y)
        inc_z_v = p.readUserDebugParameter(inc_z)

        actions.append([inc_x_v,inc_y_v,inc_z_v])
        pos_puck.append( list(p.getBasePositionAndOrientation(puck_id)[0]) )
        pos_goal.append( list(p.getBasePositionAndOrientation(goal_id)[0]) )

        moveCartesianOffset(cartesian_control_id,inc_x = inc_x_v, inc_y = inc_y_v, inc_z = inc_z_v )

        print("simulate")

        robot.step_simulation()
        for i in range(repeats-1):
            p.stepSimulation()
            time.sleep(robot.time_step)

        for i in range(1,4):
            print(p.getJointState(cartesian_control_id,i)[0])

    qKey = ord('o')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:

        actions.append([0,0,inc_value])
        pos_puck.append( list(p.getBasePositionAndOrientation(puck_id)[0]) )
        pos_goal.append( list(p.getBasePositionAndOrientation(goal_id)[0]) )
        inc_z_v = inc_value

        moveCartesianOffset(cartesian_control_id, inc_z = inc_z_v )

        robot.step_simulation()
        for i in range(repeats-1):
            p.stepSimulation()
            time.sleep(robot.time_step)

    qKey = ord('u')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:

        actions.append([0,0,-inc_value])
        pos_puck.append( list(p.getBasePositionAndOrientation(puck_id)[0]) )
        pos_goal.append( list(p.getBasePositionAndOrientation(goal_id)[0]) )
        inc_z_v = -inc_value

        moveCartesianOffset(cartesian_control_id,inc_z = inc_z_v)

        robot.step_simulation()
        for i in range(repeats-1):
            p.stepSimulation()
            time.sleep(robot.time_step)

    qKey = ord('l')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:

        actions.append([0,inc_value,0])
        pos_puck.append( list(p.getBasePositionAndOrientation(puck_id)[0]) )
        pos_goal.append( list(p.getBasePositionAndOrientation(goal_id)[0]) )
        inc_y_v = inc_value

        moveCartesianOffset(cartesian_control_id,inc_y = inc_y_v )

        robot.step_simulation()
        for i in range(repeats-1):
            p.stepSimulation()
            time.sleep(robot.time_step)

    qKey = ord('j')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:

        actions.append([0,-inc_value,0])
        pos_puck.append( list(p.getBasePositionAndOrientation(puck_id)[0]) )
        pos_goal.append( list(p.getBasePositionAndOrientation(goal_id)[0]) )
        inc_y_v = -inc_value

        moveCartesianOffset(cartesian_control_id,inc_y = inc_y_v )

        robot.step_simulation()
        for i in range(repeats-1):
            p.stepSimulation()
            time.sleep(robot.time_step)

    qKey = ord('i')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:

        actions.append([inc_value,0,0])
        pos_puck.append( list(p.getBasePositionAndOrientation(puck_id)[0]) )
        pos_goal.append( list(p.getBasePositionAndOrientation(goal_id)[0]) )
        inc_x_v = inc_value

        moveCartesianOffset(cartesian_control_id,inc_x = inc_x_v )

        robot.step_simulation()
        for i in range(repeats-1):
            p.stepSimulation()
            time.sleep(robot.time_step)

    qKey = ord('k')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:

        actions.append([-inc_value,0,0])
        pos_puck.append( list(p.getBasePositionAndOrientation(puck_id)[0]) )
        pos_goal.append( list(p.getBasePositionAndOrientation(goal_id)[0]) )
        inc_x_v = -inc_value

        moveCartesianOffset(cartesian_control_id,inc_x = inc_x_v )

        robot.step_simulation()
        for i in range(repeats-1):
            p.stepSimulation()
            time.sleep(robot.time_step)

    qKey = ord('n')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        print("save")
        actions = np.array(actions)
        #np.save("actions_fetch.npy",actions)
        np.savez("fetch_"+nExp+'.npz', actions = actions, goal = pos_goal, puck = pos_puck)

        ExperimentData = pd.DataFrame({})
        ExperimentData["j0"]=np.array(robot.database.joints_angles_rad)[:,0]
        ExperimentData["j1"]=np.array(robot.database.joints_angles_rad)[:,1]
        ExperimentData["j2"]=np.array(robot.database.joints_angles_rad)[:,2]
        ExperimentData["j3"]=np.array(robot.database.joints_angles_rad)[:,3]
        ExperimentData["j4"]=np.array(robot.database.joints_angles_rad)[:,4]
        ExperimentData["j5"]=np.array(robot.database.joints_angles_rad)[:,5]
        ExperimentData["j6"]=np.array(robot.database.joints_angles_rad)[:,6]
        #robot.database.joint_angles_vel_rad
        #robot.database.joint_torques


        ExperimentData["tcp x"]=np.array(robot.database.tcp_position)[:,0]
        ExperimentData["tcp y"]=np.array(robot.database.tcp_position)[:,1]
        ExperimentData["tcp z"]=np.array(robot.database.tcp_position)[:,2]
        #robot.database.tcp_orientation_q

        ExperimentData["rot x"]=np.array(robot.database.tcp_orientation_e)[:,0]
        ExperimentData["rot y"]=np.array(robot.database.tcp_orientation_e)[:,1]
        ExperimentData["rot z"]=np.array(robot.database.tcp_orientation_e)[:,2]

        ExperimentData.to_excel("fetch_"+nExp+".xlsx")
        break
    qKey = ord('r')
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
        print("reproduce")
        np_actions_goal_puck = np.load("Fetch_data/fetch_"+nExp+".npz")


        pos_goal = list(np_actions_goal_puck["goal"][0,:])

        pos_puck = list(np_actions_goal_puck["puck"][0,:])

        actions_numpy = np_actions_goal_puck["actions"]
        actions = []

        p.removeBody(puck_id)
        puck_id=p.loadURDF(os.path.join(root, puck_urdf), pos_puck)
        p.removeBody(goal_id)
        goal_id=p.loadURDF(os.path.join(root, goal_urdf), pos_goal)

        for i in range (actions_numpy.shape[0]):
            [inc_x_v,inc_y_v,inc_z_v] = list(actions_numpy[i,:])
            actions.append([inc_x_v,inc_y_v,inc_z_v])
            moveCartesianOffset(cartesian_control_id,inc_x = inc_x_v,inc_y = inc_y_v,inc_z = inc_z_v )

            robot.step_simulation()
            for i in range(repeats-1):
                p.stepSimulation()
                time.sleep(robot.time_step)
