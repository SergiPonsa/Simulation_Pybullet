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

# ------------------------------------------------------------------------------------------------------

if __name__ == '__main__':

    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)



    # create robot instance with default configuration
    robot = KinovaGen3()
    print("Class created")
    robot.move_home()

    #move original robot


    [home_position,home_orientation_q] = robot.get_actual_tcp_pose()
    counter_test = 10**5
    distance = 0.2
    time_home = 0.2
    time_between=0.0
    element = "friction"
    element_value = 1.0

    record_experimet_f = open("exp_"+str(element)+"_"+str(element_value)\
                            +".txt","w")

    record_experimet_f.write("#"*20)
    record_experimet_f.write("\n Original Robot with"+str(counter_test)+" steps \n")
    record_experimet_f.write("#"*20)
    record_experimet_f.write("\n"*2)
    #Initial position

    #In x
    record_experimet_f.write("Positive x "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([distance,0,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative x "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([-distance,0,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)

    #In y
    record_experimet_f.write("Positive y "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,distance,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative y "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,-distance,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)

    #In z
    record_experimet_f.write("Positive z "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,0,distance],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative z "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,0,-distance],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)

    ###############################################

    counter_test = 64

    record_experimet_f.write("#"*20)
    record_experimet_f.write("\n Original Robot with "+str(counter_test)+" steps \n")
    record_experimet_f.write("#"*20)
    record_experimet_f.write("\n"*2)
    #Initial position

    #In x
    record_experimet_f.write("Positive x "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([distance,0,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative x "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([-distance,0,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)

    #In y
    record_experimet_f.write("Positive y "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,distance,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative y "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,-distance,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)

    #In z
    record_experimet_f.write("Positive z "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,0,distance],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative z "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,0,-distance],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)


    #Modify all the links mass , don't specify the name

    file_2_write_ex1 = robot.create_empty_file(robot.robot_urdf)
    print(file_2_write_ex1)
    if (file_2_write_ex1 != "error"):
        robot.modify_urdf(robot.robot_urdf,file_2_write_ex1,element,element_value)


    #Modify only one link mass , specify the name
    # The first one it's saved to a external file, to keep the original
    if(element in robot.modify_elements_link()):
        info = p.getJointInfo(robot.robot_id,robot.last_robot_joint_index)
        LastLinkName = str(info[12], "utf-8")
        print(LastLinkName)

    file_2_write_ex2 = robot.create_empty_file(robot.robot_urdf)


    print(file_2_write_ex2)
    if (file_2_write_ex2 != "error"):
        if(element in robot.modify_elements_link()):
            robot.modify_urdf(robot.robot_urdf,file_2_write_ex2,element,element_value,\
            link_or_joint_name=LastLinkName)
        else:
            robot.Copy_file(robot.robot_urdf,file_2_write_ex2)
    print("created")

    # The second one and de following ones it's saved to the first external file, but first it's copied to a dummy
    file_dummy = robot.create_empty_file(robot.robot_urdf)
    for control_joint_name in robot.robot_control_joints :

        #Get the link name or joint name
        if(element in robot.modify_elements_link()):
            info = p.getJointInfo(robot.robot_id,robot.joints[control_joint_name].id)
            LinkName = str(info[12], "utf-8")
        else:
            LinkName = control_joint_name

        print(LinkName)

        #copy the previous to dummy
        robot.Copy_file(file_2_write_ex2,file_dummy)

        file_2_read = file_dummy

        robot.modify_urdf(file_2_read,file_2_write_ex2,element,element_value,\
        link_or_joint_name=LinkName)
    print("created")

    #Disconnect and connect again
    p.disconnect()
    time.sleep(2.0)
    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    robot = KinovaGen3(robot_urdf="models/urdf/JACO3_URDF_V11_1.urdf")

    robot.move_home()
    ###############################################

    record_experimet_f.write("#"*20)
    record_experimet_f.write("\n  Robot Modified with "+str(element)+" " \
                            +str(element_value)+" "+str(counter_test)+" steps \n")
    record_experimet_f.write("#"*20)
    record_experimet_f.write("\n"*2)
    #Initial position

    #In x
    record_experimet_f.write("Positive x "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([distance,0,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative x "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([-distance,0,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)

    #In y
    record_experimet_f.write("Positive y "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,distance,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative y "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,-distance,0],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)

    #In z
    record_experimet_f.write("Positive z "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,0,distance],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")

    time.sleep(time_home)
    robot.move_home()

    record_experimet_f.write("Negative z "+ str(distance)+" cm \n")
    robot.move_cartesian_offset([0,0,-distance],[0,0,0],counter_max = counter_test)
    [actual_position_test,actual_orientation_q_test] = robot.get_actual_tcp_pose()
    diference = []
    for i,j in zip(actual_position_test, home_position):
        diference.append(i-j)
    record_experimet_f.write( str( diference ) + "\n" )
    record_experimet_f.write( str( robot.get_angle_diference(robot.home_angles,data_by_joint=True)) + "\n")
    record_experimet_f.write("\n"*2)

    time.sleep(time_home)
    robot.move_home()

    time.sleep(time_between)

    record_experimet_f.close()
    p.disconnect()



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
