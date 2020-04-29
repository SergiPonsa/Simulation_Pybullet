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

def Do_Experiment(repeats,experiment,robot,max_vel=30,force_per_one=1,joint = 1,kp=0.1,ki=0.0,kd=0.0):
    for iteration in range(repeats):
        # Initialization
        counter = simSteps(experiment,timestep) # detemine time

        #create PIDs
        PID_List = []
        for i in range(len(robot.robot_control_joints) ):
            PID_List.append( PID(max_velocity=max_vel,kp=kp,ki=ki,kd=kd) )
        robot.database_name = "Data_"+str(iteration)

        #Move to joint 0
        angles_zero = [0.0]*len(robot.robot_control_joints)
        print(angles_zero)
        robot.save_database = False
        robot.move_joints(joint_param_value = angles_zero, wait=True,desired_force_per_one=force_per_one)

        #Start saving data every time step
        robot.save_database = True
        #time.sleep(10)
        #Execute experiment during the especified time
        for simStep in range(counter):

            #Every step compute the pid
            PID_List = set_target_thetas(counter, PID_List,experiment,"PyBullet",simStep,joint)
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

    #Change one lasttime the name and simulate to append it to the database list
    robot.database_name = "dummy"
    robot.step_simulation()

    return robot

def PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title):
    dataframe_list = []
    angles_list = []

    #Go through all the database classes created during the experiment
    for data in robot.database_list:

        #pass the data to a list to do the average
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

        #create the data frame
        aux_df = pd.DataFrame({})
        #convert data to numpy
        joint_angles = np.array(data.joints_angles_rad)
        for i in range(len(robot.robot_control_joints)):
            column_name = "joint"+str(i)

            aux_df[column_name] = joint_angles [:,i]
        aux_df.index = data.time
        dataframe_list.append(aux_df)

    #create and excel with all the experiments
    # Create a Pandas Excel writer using XlsxWriter as the engine.
    writer = pd.ExcelWriter(folder+"/"+title+"_all_experiments.xlsx", engine='xlsxwriter')
    for count in range ( len(dataframe_list) ):
        #dataframe_list[count].head
        dataframe_list[count].to_excel(writer, sheet_name='Sheet'+str(count))

    # Close the Pandas Excel writer and output the Excel file.
    writer.save()

    #compute the average, convert all the data to numpy to make it easier
    joint_angles_array = np.array(angles_list)
    print(joint_angles_array.shape)
    #dimensions of the numpy
    [experiments,steps,joints] = joint_angles_array.shape

    average_steps = []
    for stp in range(steps):
        average_joints = []
        for j in range(joints):
            average_value = np.average(joint_angles_array[:,stp,j])
            average_joints.append(average_value)
        average_steps.append(average_joints)

    #create the average data frame
    avg_df = pd.DataFrame({})

    joint_angles_average = np.array(average_steps)
    for i in range(len(robot.robot_control_joints)):
        column_name = "joint"+str(i)

        avg_df[column_name] = joint_angles_average [:,i]
    avg_df.index = data.time
    avg_df.to_excel(folder+"/"+title+"_average.xlsx", sheet_name='Sheet1')

def SubstractExcel_2_excel(path_Excel_to_substract,path_Excel_substract,title,folder):
    DF_ToSubstract = pd.read_excel(path_Excel_to_substract)
    DF_Substract = pd.read_excel(path_Excel_substract)

    np_ToSubstract = DF_ToSubstract.to_numpy()
    np_Substract = DF_Substract.to_numpy()

    if (np_Substract.shape != np_ToSubstract.shape):

        print("Data samples have diferent shapes correct it")


    np_result = np_ToSubstract[:,1:] - np_Substract[:,1:]
    np_index = np_ToSubstract[:,:1] - np_Substract[:,:1]

    print(len(list(np_index)))

    DF_Result = pd.DataFrame({})
    columns = list(DF_ToSubstract.columns)

    print(columns)
    column = columns.pop(0)
    for j in range(len(columns)):
        print(j)
        column = columns.pop(0)
        DF_Result[column] = np_result[:,j]
    newindex = xnew = np.linspace(0, (1/256) *len(list(np_index)), len(list(np_index)))
    DF_Result.index = newindex


    #DF_Result = pd.DataFrame(data=np_result,index=DF_ToSubstract.index,columns=DF_ToSubstract.index)  # 1st row as the column names

    DF_Result.to_excel(folder+"/"+title+"_substract.xlsx", sheet_name='Sheet1')





experiment = "Single"
#experiment = "Double"
#experiment = "Sergi"
#experiment = "Cube"
timestep = 1./240.
force_per_one = 1.0
max_vel = 30

repeats = 20
folder = "Experiments_"+experiment
title = "Original"

if __name__ == '__main__':

    #Connect to pybullet
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    #create the robot
    robot = KinovaGen3()
    #robot = KinovaGen3(robot_urdf = "models/urdf/JACO3_URDF_V11modpaper.urdf")

    #Decide to wait the real time
    robot.visual_inspection = False

    if (experiment == "Single"):
        for i in range(7):
            joint = i
            robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one,joint)
            title = "original"+"_"+experiment+"_"+str(joint)
            PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)
            p.disconnect()
            time.sleep(0.1)
            #Connect to pybullet
            p.connect(p.GUI)
            p.setGravity(0.0, 0.0, -9.81)

            #create the robot
            robot = KinovaGen3()
            robot.visual_inspection = False
    else:
        #Original
        robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one)
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)


    #Modified
    element_to_modify_list = [["damping"],["damping"],["damping"],["damping"],["damping"],\
                            ["mass"],["mass"],["mass"],["mass"],\
                            ["inertia"],["inertia"],["inertia"],["inertia"]]
    x = len(robot.robot_control_joints)
    element_to_modify_value_list =[[1.0]*x,[15.5]*x,[7.25]*x,[10.875]*x,[3.625]*x,\
                                    [1.5]*x,[2.5]*x,[3.5]*x,[4.5]*x,\
                                    [10.0**-4,2*10.0**-4,3*10.0**-4]*x,\
                                    [5*10.0**-4,3*5*10.0**-4,2*5*10.0**-4]*x,\
                                    [2*10.0**-5,10.0**-5,3*10.0**-5]*x,\
                                    [2*10.0**-6,3*10.0**-6,10.0**-6]*x]


    #Start experiment
    #create the robot
    for element in element_to_modify_list:
        element_values = element_to_modify_value_list.pop(0)
        print(element_values)
        #time.sleep(10)

        p.disconnect()
        time.sleep(0.1)
        p.connect(p.GUI)
        p.setGravity(0.0, 0.0, -9.81)
        robot = KinovaGen3()


        #Decide to wait the real time
        robot.visual_inspection = False
        #modify some parameter of the robot
        #robot.modify_robot_pybullet(robot.robot_control_joints,["inertia"],[100.0,100.0,100.0]*len(robot.robot_control_joints))
        title = str(element)+"_"+str(element_values)

        robot.modify_robot_pybullet(robot.robot_control_joints,element,element_values)
        time.sleep(10.0)

        #Repeat it repeats times
        robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one)
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)
        path_Excel_to_substract = "./"+folder+"/"+title+"_average.xlsx"
        path_Excel_substract = "./"+folder+"/"+"Original"+"_average.xlsx"
        SubstractExcel_2_excel(path_Excel_to_substract,path_Excel_substract,title,folder)

    velocities = [20,40,50,100]
"""
    for velocity in velocities:
        max_vel = velocity

        p.disconnect()
        time.sleep(0.1)
        p.connect(p.GUI)
        p.setGravity(0.0, 0.0, -9.81)
        robot = KinovaGen3()


        #Decide to wait the real time
        robot.visual_inspection = False
        title = "Max_velocity"+"_"+str(max_vel)

        #Repeat it repeats times
        robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one)
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)
        path_Excel_to_substract = "./"+folder+"/"+title+"_average.xlsx"
        path_Excel_substract = "./"+folder+"/"+"Original"+"_average.xlsx"
        SubstractExcel_2_excel(path_Excel_to_substract,path_Excel_substract,title,folder)

        max_vel = 30

    kps = [0.1,0.05,0.025,0.2,0.4,0.6,0.8,1.6]

    for v_kp in kps:
        kp = v_kp

        p.disconnect()
        time.sleep(0.1)
        p.connect(p.GUI)
        p.setGravity(0.0, 0.0, -9.81)
        robot = KinovaGen3()


        #Decide to wait the real time
        robot.visual_inspection = False
        title = "kp"+"_"+str(kp)

        #Repeat it repeats times
        robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one,kp=kp)
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)
        path_Excel_to_substract = "./"+folder+"/"+title+"_average.xlsx"
        path_Excel_substract = "./"+folder+"/"+"Original"+"_average.xlsx"
        SubstractExcel_2_excel(path_Excel_to_substract,path_Excel_substract,title,folder)

        kp=0.1


    kds = [0.1,0.05,0.025,0.2,0.4,0.6,0.8,1.6,10,50]

    for v_kd in kds:
        kd = v_kd

        p.disconnect()
        time.sleep(0.1)
        p.connect(p.GUI)
        p.setGravity(0.0, 0.0, -9.81)
        robot = KinovaGen3()


        #Decide to wait the real time
        robot.visual_inspection = False
        title = "kd"+"_"+str(kd)

        #Repeat it repeats times
        robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one,kd=kd)
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)
        path_Excel_to_substract = "./"+folder+"/"+title+"_average.xlsx"
        path_Excel_substract = "./"+folder+"/"+"Original"+"_average.xlsx"
        SubstractExcel_2_excel(path_Excel_to_substract,path_Excel_substract,title,folder)

        kd=0.0

    kis = [0.1,0.05,0.025,0.2,0.4,0.6,0.8,1.6]

    for v_ki in kis:
        ki = v_ki

        p.disconnect()
        time.sleep(0.1)
        p.connect(p.GUI)
        p.setGravity(0.0, 0.0, -9.81)
        robot = KinovaGen3()


        #Decide to wait the real time
        robot.visual_inspection = False
        title = "ki"+"_"+str(ki)

        #Repeat it repeats times
        robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one,ki=ki)
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)
        path_Excel_to_substract = "./"+folder+"/"+title+"_average.xlsx"
        path_Excel_substract = "./"+folder+"/"+"Original"+"_average.xlsx"
        SubstractExcel_2_excel(path_Excel_to_substract,path_Excel_substract,title,folder)

        ki=0.0

    test_force_per_one = [0.75,0.625,0.56,0.5]

    for force_x_one in test_force_per_one:
        force_per_one = force_x_one

        p.disconnect()
        time.sleep(0.1)
        p.connect(p.GUI)
        p.setGravity(0.0, 0.0, -9.81)
        robot = KinovaGen3()


        #Decide to wait the real time
        robot.visual_inspection = False
        title = "Force_x_one"+"_"+str(force_per_one )

        #Repeat it repeats times
        robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one)
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)
        path_Excel_to_substract = "./"+folder+"/"+title+"_average.xlsx"
        path_Excel_substract = "./"+folder+"/"+"Original"+"_average.xlsx"
        SubstractExcel_2_excel(path_Excel_to_substract,path_Excel_substract,title,folder)

        force_per_one  = 1

    #create urdf to modify
    file_2_write_ex2 = robot.create_empty_file(robot.robot_urdf)
    robot.Copy_file(robot.robot_urdf,file_2_write_ex2)
    friction_values = [1.0,2.0,10.0,0.5,100.0,10**5]

    for friction in friction_values :
        friction_v = friction
        robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["friction"],\
                                                                                        [friction_v,friction_v,friction_v,friction_v,friction_v,friction_v,friction_v]  )
        #time.sleep(10.0)
        p.disconnect()
        time.sleep(0.1)
        p.connect(p.GUI)
        p.setGravity(0.0, 0.0, -9.81)

        title = "Friction"+"_"+str(friction)

        robot = KinovaGen3(robot_urdf=file_2_write_ex2)
        robot.visual_inspection=False
        robot = Do_Experiment(repeats,experiment,robot,max_vel,force_per_one)
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title)
        path_Excel_to_substract = "./"+folder+"/"+title+"_average.xlsx"
        path_Excel_substract = "./"+folder+"/"+"Original"+"_average.xlsx"
        SubstractExcel_2_excel(path_Excel_to_substract,path_Excel_substract,title,folder)
    """
