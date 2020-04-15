import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict

from Rotations import *
from KinovaGen3Class import KinovaGen3

import pandas as pd

#experiment = "Single"
experiment = "Double"
#experiment = "Cube"
timestep = 1./240.
force_per_one = 1.0
max_vel = 30

repeats = 20
folder = "Experiments"
title = "Inertia_100"

if __name__ == '__main__':
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)
    robot = KinovaGen3()
    robot.visual_inspection = False

    robot.modify_robot_pybullet(robot.robot_control_joints,["inertia"],[100.0,100.0,100.0]*len(robot.robot_control_joints))
    time.sleep(10.0)
    for iteration in range(repeats):
        # Initialization
        counter = simSteps(experiment,timestep)
        PID_List = []
        for i in range(len(robot.robot_control_joints) ):
            PID_List.append( PID(max_vel) )
        robot.database_name = "Data_"+str(iteration)
        angles_zero = [0.0]*len(robot.robot_control_joints)
        print(angles_zero)
        robot.save_database = False
        robot.move_joints(joint_param_value = angles_zero, wait=True,desired_force_per_one=force_per_one)
        robot.save_database = True
        #time.sleep(10)
        for simStep in range(counter):

            #Every step compute the pid
            PID_List = set_target_thetas(counter, PID_List,experiment,"PyBullet",simStep)
            print
            #Every 12 steeps apply the control
            if simStep % 12 == 0:
                current_angles=robot.get_actual_control_joints_angle()
                velocity_angles= []

                #Compute the velocity
                for i in range( len(robot.robot_control_joints ) ):
                    print(i)
                    print(PID_List[i].get_target_theta())
                    print(current_angles)
                    velocity_angles.append( PID_List[i].get_velocity(math.degrees(current_angles[i])) /57.32484076 )

                #Apply the controls
                robot.move_joints_control_vel( joint_param_value = velocity_angles ,wait = False , desired_force_per_one=force_per_one)

            robot.step_simulation()

    robot.database_name = "dummy"
    robot.step_simulation()

    dataframe_list = []
    angles_list = []
    for data in robot.database_list:
        angles_list.append(data.joints_angles_rad)

        print(data.name)
        print("\n")
        """
        aux_df = pd.DataFrame({ "joint_angles_rad": data.joints_angles_rad,\
                                "joint_angles_vel_rad": data.joint_angles_vel_rad,\
                                "joint_torques": data.joint_torques,\
                                "tcp_position": data.tcp_position,\
                                "tcp_orientation_q": data.tcp_orientation_q,\
                                "tcp_orientation_e": data.tcp_orientation_e,\
                                "time": data.time})
        """

        aux_df = pd.DataFrame({})
        joint_angles = np.array(data.joints_angles_rad)
        for i in range(len(robot.robot_control_joints)):
            column_name = "joint"+str(i)

            aux_df[column_name] = joint_angles [:,i]
        aux_df.index = data.time
        dataframe_list.append(aux_df)

    # Create a Pandas Excel writer using XlsxWriter as the engine.
    writer = pd.ExcelWriter(folder+"/"+title+"_all_experiments.xlsx", engine='xlsxwriter')
    for count in range ( len(dataframe_list) ):
        #dataframe_list[count].head
        dataframe_list[count].to_excel(writer, sheet_name='Sheet'+str(count))

    # Close the Pandas Excel writer and output the Excel file.
    writer.save()

    #compute the average
    joint_angles_array = np.array(angles_list)
    print(joint_angles_array.shape)
    [experiments,steps,joints] = joint_angles_array.shape

    average_steps = []
    for stp in range(steps):
        average_joints = []
        for j in range(joints):
            average_value = np.average(joint_angles_array[:,stp,j])
            average_joints.append(average_value)
        average_steps.append(average_joints)

    avg_df = pd.DataFrame({})

    joint_angles_average = np.array(average_steps)
    for i in range(len(robot.robot_control_joints)):
        column_name = "joint"+str(i)

        avg_df[column_name] = joint_angles_average [:,i]
    avg_df.index = data.time
    avg_df.to_excel(folder+"/"+title+"_average.xlsx", sheet_name='Sheet1')
