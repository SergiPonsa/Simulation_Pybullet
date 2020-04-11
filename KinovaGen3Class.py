# name: Kinova-Gen3-Class.py
# content: Kinova Gen3 robot children of Robot class
# # author: BitMetrics (Sergi Ponsà)
# date: 03-2020
# ------------------------------------------------------------------------------------------------------

import os
import time

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
    home_angles = [0, 0.392, 0.0, 1.962, 0.0, 0.78, 0.0],
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

    def Write_modification_test_offset(self,distance_to_test,element_to_modify,element_value_to_modify,counter_test = 10**5,title="",time_home = 0,time_between=0.0):

        #create file with the results
        record_experimet_f = open("Experiments/"+title+"exp_"+str(element_to_modify,)+"_"+str(element_value_to_modify) +\
                                "_steps_"+str(counter_test)+".txt","w")

        #Initial position
        robot.move_home()


        dirrection = ["x","y","z"]
        sign = [1,-1]
        dict_pos ={"x":0,\
                    "y":1,\
                    "z":2}
        dict_sig ={-1:"Negative",\
                    1:"Positive"}

        record_experimet_f.write(title)
        record_experimet_f.write("\n"*2)
        for i in dirrection:
            for j in sign:

                record_experimet_f.write(dict_sig[j]+" "+ i +" "+ str(distance_to_test)+" cm \n")
                offset = [0,0,0]
                print(offset)
                offset[dict_pos[i]] = j * distance_to_test
                print(offset)
                [target_pos,target_orien_q] = robot.get_cartesian_offset_target_pose(offset,[0,0,0])

                robot.move_cartesian_offset(offset,[0,0,0],counter_max = counter_test)

                [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()

                difference = robot.get_pose_diference(actual_position_test,p.getEulerFromQuaternion(actual_orientation_q_test),\
                                        target_pos,p.getEulerFromQuaternion(target_orien_q))

                record_experimet_f.write( "Target position"+ "\n" )
                record_experimet_f.write( str( target_pos ) + "\n" )
                record_experimet_f.write( "Final position"+ "\n" )
                record_experimet_f.write( str( actual_position_test ) + "\n" )
                record_experimet_f.write( "Difference with the target position"+ "\n" )
                record_experimet_f.write( str( difference ) + "\n" )
                record_experimet_f.write( "Difference with the target angles" + "\n" )
                record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
                record_experimet_f.write("\n" * 2)

                time.sleep(time_home)
                robot.move_home()

            time.sleep(time_between)
        record_experimet_f.close()

# ------------------------------------------------------------------------------------------------------

if __name__ == '__main__':

    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)



    # create robot instance with default configuration
    robot = KinovaGen3()
    print("Class created")
    robot.move_home()
    element_to_modify = ["mass","friction"]
    element_value_to_modify = [1.0,2.0]
    distance = 0.2

    robot.Write_modification_test_offset(distance,element_to_modify,element_value_to_modify,counter_test = 10**5,title="Original Robot",time_home = 2.0,time_between=1.0)
    robot.Write_modification_test_offset(distance,element_to_modify,element_value_to_modify,counter_test = 64,title="Original Robot",time_home = 2.0,time_between=1.0)

    ###############################################
    #Modify all the links mass , don't specify the name

    file_2_write_ex1 = robot.create_empty_file(robot.robot_urdf)
    print(file_2_write_ex1)
    if (file_2_write_ex1 != "error"):
        robot.modify_urdf(robot.robot_urdf,file_2_write_ex1,"mass",3.0)
    print("Created")
    time.sleep(10.0)

    #Modify only one link mass , specify the name
    # The first one it's saved to a external file, to keep the original
    file_2_write_ex2 = robot.create_empty_file(robot.robot_urdf)
    robot.Copy_file(robot.robot_urdf,file_2_write_ex2)

    ####################################################################
    #Mass test
    mass_v=2.0
    robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["mass"],\
                                                                                        [mass_v,mass_v,mass_v,mass_v,mass_v,mass_v,mass_v] )
    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf=file_2_write_ex2)

    robot.move_home()
    robot.Write_modification_test_offset(distance,"mass",mass_v,counter_test = 64,title="Modified Robot",time_home = 0.1,time_between=0.1)

    """
    ####################################################################
    #Inertia test
    inertia_v = 1.0
    robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["inertia"],\
                                                                                        [[inertia_v,0.0,0.0,inertia_v,0.0,inertia_v]*7])
    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf=file_2_write_ex2)

    robot.move_home()
    robot.Write_modification_test_offset(distance,"inertia",1.0,counter_test = 64,title="Modified Robot",time_home = 0.1,time_between=0.1)
    ###############################################
    #Damping test
    damping_v = 1.0
    robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["damping"],\
                                                                                        [damping_v,damping_v,damping_v,damping_v,damping_v,damping_v,damping_v] )
    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf=file_2_write_ex2)

    robot.move_home()
    robot.Write_modification_test_offset(distance,"damping",1.0,counter_test = 64,title="Modified Robot",time_home = 0.1,time_between=0.1)
    ###############################################
    #friction test
    friction_v = 1.0
    robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["friction"],\
                                                                                        [friction_v,friction_v,friction_v,friction_v,friction_v,friction_v,friction_v]  )
    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf=file_2_write_ex2)

    robot.move_home()
    robot.Write_modification_test_offset(distance,"friction",1.0,counter_test = 64,title="Modified Robot",time_home = 0.1,time_between=0.1)
    ###############################################
    #lower test
    lower_v=1.57
    robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["lower"],\
                                                                                        [lower_v,lower_v,lower_v,lower_v,lower_v,lower_v,lower_v]  )
    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf=file_2_write_ex2)

    robot.move_home()
    robot.Write_modification_test_offset(distance,"lower",1.0,counter_test = 64,title="Modified Robot",time_home = 0.1,time_between=0.1)
    ###############################################
    #upper test
    upper_v = 1.57
    robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["upper"],\
                                                                                        [upper_v,upper_v,upper_v,upper_v,upper_v,upper_v,upper_v]  )
    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf=file_2_write_ex2)

    robot.move_home()
    robot.Write_modification_test_offset(distance,"upper",1.0,counter_test = 64,title="Modified Robot",time_home = 0.1,time_between=0.1)
    ###############################################
    #effort test
    effort_v = 39.0
    robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["effort"],\
                                                                                        [effort_v,effort_v,effort_v,effort_v,effort_v,effort_v,effort_v]  )
    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf=file_2_write_ex2)

    robot.move_home()
    robot.Write_modification_test_offset(distance,"effort",1.0,counter_test = 64,title="Modified Robot",time_home = 0.1,time_between=0.1)
    ###############################################
    #velocity test
    velocity_v = 0.8727
    robot.modify_urdf_list(file_2_write_ex2,file_2_write_ex2,robot.robot_control_joints,["velocity"],\
                                                                                        [velocity_v,velocity_v,velocity_v,velocity_v,velocity_v,velocity_v,velocity_v]  )
    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf=file_2_write_ex2)

    robot.move_home()
    robot.Write_modification_test_offset(distance,"velocity",1.0,counter_test = 64,title="Modified Robot",time_home = 0.1,time_between=0.1)
    ###import pdb############################################

    p.disconnect()
    """


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
