# name: ur5_rf85.py
# content: robot+gripper class module
# author: BitMetrics (David Picado)
# date: 09-2019 (file created)
# Content Modification : ur5+gripper_rf85 class module (Modification of robot+gripper)
# author: BitMetrics (Sergi Ponsà)
# date: 11-2019 (file modified)
# ------------------------------------------------------------------------------------------------------

import os
import time
import pdb
import math
import numpy as np
import pybullet as p
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict



# ------------------------------------------------------------------------------------------------------

class Robot():
    """Generic Robot class"""

    def __init__(self,
                urdf_root = pybullet_data.getDataPath(),
                root = "",
                robot_urdf = "",

                robot_launch_pos = [0, 0, 0],
                robot_launch_orien = p.getQuaternionFromEuler([0, 0, 0]),

                last_robot_joint_name = "",
                robot_control_joints = [],
                robot_mimic_joints_name = [],
                robot_mimic_joints_master = [],
                robot_mimic_multiplier = [],
                tool_orient_e = [3.14,0,0],

                nullspace = True,
                home_angles = [-0.992, -1.157, 1.323, -1.720, -1.587, 0.0],
                visual_inspection = True,

                tcp_offset_pos = [0.0, 0.0, 0.0],
                tcp_offset_orien_e = [0.0, 0.0, 0.0]):


        """Initialization function

        urdf_root (str): root for URDF objects
        root (str): root directory for local files
        robot_urdf (str): name and location of robot URDF file,

        robot_launch_pos (list double | x,y,z in m): determine the robot base position
        robot_launch_orien (list double | rx,ry,rz in radiants): determine the robot base orientation


        last_robot_joint_name (str): URDF name of the las robot joint, the id it's the same than the last robot link
        robot_control_joints (list of str): name robot control joints in right order for inverse kinematics
        robot_mimic_joints_name (list of str): name robot joints which follow control joints in order to control them
        robot_mimic_joints_master (list of str): name joints which master each mimic joint,
        robot_mimic_multiplier (list of doubles): value of increase of the mimic respect the by increase of the master

        tcp_offset_pos (list of doubles | x,y,z in m): position offset referent to the robot_conection_to_tool_name
        tcp_offset_orien (list of doubles | rx,ry,rz in rad RPY): position offset referent to the robot_conection_to_tool_name

        nullspace (boolean): True if we want to find the inverse kinematicscloser to home
        home_position (list of doubles): joint angles in radiants of the initial joint position, I was also useing it to find solution near this configuration
        visual_inspection=True)

        """

        self.root = root
        self.urdf_root = urdf_root
        self.robot_urdf = os.path.join(root, robot_urdf)

        self.robot_launch_pos = robot_launch_pos
        self.robot_launch_orien = robot_launch_orien

        self.last_robot_joint_name = last_robot_joint_name
        self.robot_control_joints = robot_control_joints
        self.robot_mimic_joints_name = []
        self.robot_mimic_joints_master = []
        self.robot_mimic_multiplier = []
        self.tool_orient_e = tool_orient_e


        self.nullspace = nullspace
        self.home_angles = home_angles
        self.visual_inspection = visual_inspection

        self.tcp_offset_pos = tcp_offset_pos
        self.tcp_offset_orien_e = tcp_offset_orien_e


        self.opening_length = 0.085  # start with the gripper open

        print(self.robot_urdf)

        # launch robot in the world
        self.robot_id = p.loadURDF(os.path.join(root, robot_urdf), robot_launch_pos, robot_launch_orien,flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT )
        print("Robot launched")

        # robot data structure
        joint_type_list = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]

        joint_info = namedtuple("jointInfo",
                               ["id", "name", "type","damping","friction","lower_limit", "upper_limit", "max_force", "max_velocity"])

        self.joints = AttrDict()
        self.joint_names = []

        # get robot data from urdf

        #get data of the joints
        #the id of the joint it's the same than their children link
        self.num_joints = p.getNumJoints(self.robot_id)
        # initialize variables
        self.robot_control_joints_index = np.zeros(self.num_joints, dtype=int)  # to avoid errors
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_id = info[0]
            joint_name = info[1].decode("utf-8")
            self.joint_names.append(joint_name) # I use it to search info in the dicctionary
            joint_type = joint_type_list[info[2]]
            joint_damping = info[6]
            joint_friction = info[7]
            joint_lower_limit = info[8]
            joint_upper_limit = info[9]
            joint_max_force = info[10]
            joint_max_velocity = info[11]
            single_info = joint_info(joint_id, joint_name, joint_type,joint_damping,joint_friction, joint_lower_limit, joint_upper_limit, joint_max_force,
                                   joint_max_velocity)
            self.joints[single_info.name] = single_info


            if joint_name == self.last_robot_joint_name:
                self.last_robot_joint_index = joint_id

            #while we get data of the joints i get the index of the control joints
            for k in range(len(self.robot_control_joints)):
                if (joint_name == self.robot_control_joints[k]):
                    self.robot_control_joints_index[k] = i


        # Null space parameters

        # parameters for the nullspace
        ll = [] #lower limit
        ul = [] #upper limit
        jr = [] # joint variance range
        rp = [] # the value it search to be closer to, the inverse kinematics

        # get robot data from the dicctionary
        numJoints = p.getNumJoints(self.robot_id)
        for i in range(numJoints):
            ll.append(self.joints[self.joint_names[i]].lower_limit)
            ul.append(self.joints[self.joint_names[i]].upper_limit)
            jr.append(
                abs(self.joints[self.joint_names[i]].lower_limit) + abs(self.joints[self.joint_names[i]].upper_limit))
            rp.append(0)

        # Tell that the solution has to be near to the home position
        for i in range(len(self.home_angles)):
            rp[i] = self.home_angles[i]

        self.lower_limit=ll
        self.upper_limit=ul
        self.joint_range=jr
        self.resting_pose=rp

    def move_joints(self, joint_param_value = None, desired_force_per_one = 1, desired_vel_per_one = 1 , wait=True, counter_max = 10**4, error_threshold = 10 ** -3):
        """Class method to control robot position by passing joint angles
        joint_param_value (list): joint angles aimed to reach
        desired_force_per_one (double): the value in per 1 of the maximum joint force  to be applied
        desired_vel_per_one (double): the value in per 1 of the maximum joint velocity to be applied
        wait (boolean): if we want to apply the control until the error is greater to the error threshold
                        or the control it's applied more than counter_max times
        counter_max: To apply maximum this amount of times the control
        error_threshold: The acceptable difference between the robot joints and the target joints
        """

        if (joint_param_value == None):
            joint_param_value = self.home_angles


        reached = False
        counter = 0
        while not reached:

            counter += 1
            # Define the control to be applied
            for i in range(len(self.robot_control_joints)):

                #Control Joints
                p.setJointMotorControl2(self.robot_id, self.joints[self.robot_control_joints[i]].id,
                                        p.POSITION_CONTROL, targetPosition = joint_param_value[i],
                                        force = self.joints[self.robot_control_joints[i]].max_force * desired_force_per_one,
                                        maxVelocity = self.joints[self.robot_control_joints[i]].max_velocity * desired_vel_per_one)
                #Mimic joints
                if (len(self.robot_mimic_joints_name)>0):
                    for j in range(len(self.robot_mimic_joints_name)):
                        follow_joint = self.joints[self.robot_mimic_joints_name[j]]
                        master_joint = self.joints[self.robot_mimic_joints_master[j]]

                        if (master_joint == self.robot_control_joints[i]):

                            p.setJointMotorControl2(self.robot_id, joint.id, p.POSITION_CONTROL,
                                                    targetPosition = joint_param_value[i] * self.robot_mimic_multiplier[i],
                                                    force = follow_joint.max_force * desired_force_per_one,
                                                    maxVelocity = follow_joint.max_velocity * desired_vel_per_one,
                                                    positionGain = 1,
                                                    velocityGain = 1)

            #If we apply the control without care if another action modify it's trajectory and apply only 1 simulation
            if wait:
                # make step simulation
                self.step_simulation()
                # check position reached
                for i in range(len(self.robot_control_joints)):
                    jointstate_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
                    if i == 0:
                        jointstatepos = [jointstate_aux[0]]
                        jointdiff = abs(jointstatepos[i] - joint_param_value[i])
                    else:
                        jointstatepos.append(jointstate_aux[0])
                        jointdiff = jointdiff + abs(jointstatepos[i] - joint_param_value[i])
                if (jointdiff <= error_threshold) or (counter > counter_max):
                    reached = True
                if (counter > counter_max):
                    print("maximum iterations reach")
            else:
                reached = True

    def get_actual_control_joints_angle(self):
        for i in range(len(self.robot_control_joints)):
            joint_state_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            if i == 0:
                joint_state_pos = [jointstate_aux[0]]  # these indexes are: [0]change joint anglesPosition, [1]Speed, [2]Reactive str, [3]Torque
            else:
                joint_state_pos.append(jointstate_aux[0])
        return joint_state_pos

    def get_actual_control_joints_velocity(self):
        for i in range(len(self.robot_control_joints)):
            joint_state_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            if i == 0:
                joint_state_velocity = [joint_state_aux[1]]  # these indexes are: [0]change joint anglesPosition, [1]Speed, [2]Reactive str, [3]Torque
            else:
                joint_state_velocity.append(joint_state_aux[1])
        return joint_state_velocity

    def get_actual_control_joints_torque(self):
        for i in range(len(self.robot_control_joints)):
            joint_state_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            if i == 0:
                joint_state_torque = [joint_state_aux[3]]  # these indexes are: [0]change joint anglesPosition, [1]Speed, [2]Reactive str, [3]Torque
            else:
                joint_state_torque.append(joint_state_aux[3])
        return joint_state_torque

    def get_actual_control_joints_full(self):
        jointsdata=[0]*3*len(self.robot_control_joints)
        for i in range(len(self.robot_control_joints)):
            jointstate_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            for j,k in enumerate([0,1,3]):
                jointsdata[i*3+j] = jointstate_aux[k]
        return jointsdata

    def move_cartesian(self, pose, max_iterations = 1000 ,nullspace = None, desired_force_per_one = 1, desired_vel_per_one = 1 , wait = True, counter_max = 10**4, error_threshold = 10 ** -3):

        if (nullspace == None):
            nullspace = self.nullspace

        """Class method to control the robot position by passing space coordinates
         and orientation and working out the correspondence to joint angles
         to call 'move_joints'

        pose (list): pose, i.e., position + orientation (as roll, pitch, yaw)
        max_iterations (int): maximum number of iterations to solve the inverse kinematics
        nullspace (boolean): find the nearest to the defined position and limits or the nearest to the actual position

        The rest of parameters are the move_joints function parameters
        """

        # Equations to ensure it's inside the reachable area (NOT NEEDED!)

        if (nullspace == True):

            inv_result = p.calculateInverseKinematics(self.robot_id, self.last_robot_joint_index, pose[0], pose[1],
                                                      maxNumIterations = max_iterations,
                                                      lowerLimits = self.lower_limit,
                                                      upperLimits = self.upper_limit,
                                                      jointRanges = self.joint_range,
                                                      restPoses = self.resting_pose)
        else:
            inv_result = p.calculateInverseKinematics(self.robot_id, self.last_robot_joint_index, pose[0], pose[1],
                                                      maxNumIterations = max_iteration)
        joint_param_value = list(inv_result)

        # perform control action with 'joint_param_value'
        self.move_joints(joint_param_value = joint_param_value, wait = wait,desired_force_per_one=desired_force_per_one,desired_vel_per_one=desired_vel_per_one,counter_max=counter_max,error_threshold=error_threshold)

    def move_home(self):
        """Class method that sends robot to 'home' position"""
        self.move_joints(joint_param_value = self.home_angles)

    def get_actual_tcp_pose(self, print_value=False,referent_to_base = False):

        last_robot_link_info = p.getLinkState(self.robot_id, self.last_robot_joint_index)
        world_last_robot_link_position = last_robot_link_info[0]
        world_last_robot_link_orientation_q = last_robot_link_info[1]
        world_last_robot_link_orientation_e = p.getEulerFromQuaternion(last_robot_link_info[1])

        if print_value:
            print("world's last robot joint position", world_last_robot_link_position,
                  world_last_robot_link_orientation_e)

        #Apply the rotation of the TCP, (the base of the TCP)
        last_robot_link_tcp_base_position = [0, 0, 0]
        last_robot_link_tcp_base_orientation_e = [self.tcp_offset_orien_e[0],
                                                  self.tcp_offset_orien_e[1],
                                                  self.tcp_offset_orien_e[2]]
        last_robot_link_tcp_base_orientation_q = p.getQuaternionFromEuler(last_robot_link_tcp_base_orientation_e)

        # transform from TCP base to world 0,0,0
        world_tcp_base_pose = p.multiplyTransforms(world_last_robot_link_position, world_last_robot_link_orientation_q,
                                                   last_robot_link_tcp_base_position,
                                                   last_robot_link_tcp_base_orientation_q)
        #Apply the translation to the tcp point
        tcp_base_tcp_end_position = self.tcp_offset_pos
        tcp_base_tcp_end_orientation_q = p.getQuaternionFromEuler([0, 0, 0])

        # transform from TCP end to world 0,0,0
        world_tcp_end_pose = p.multiplyTransforms(world_tcp_base_pose[0], world_tcp_base_pose[1],
                                                  tcp_base_tcp_end_position, tcp_base_tcp_end_orientation_q)


        if print_value:
            print("\n", "world to tcp end position", world_tcp_end_pose[0],
                     p.getEulerFromQuaternion(world_tcp_end_pose[1]))
        return world_tcp_end_pose

    def get_robot_base_pose_from_world_pose(world_position,world_orientation_q):

        [world_robot_base_position, world_robot_base_orientation_q] = p.getBasePositionAndOrientation(self.robot_id)
        [robot_base_world_position, robot_base_world_orientation_q] = p.invertTransform (world_robot_base_position, world_robot_base_orientation_q)

        return p.multiplyTransforms(robot_base_world_position, robot_base_world_orientation_q,world_position,world_orientation_q)

    def get_actual_tcp_pose_world_oriented(self, print_value=False):
        """ I want world_tcp_end_worldoriented_pose = world_tcp_end_pose *tcp_end_tcp_end_worldoriented_pose
         object position and TCP position are the same, the only difference is a rotation """

        world_tcp_end_pose = self.get_actual_tcp_pose()
        world_tcp_end_worldoriented_pose = p.multiplyTransforms(world_tcp_end_pose[0], world_tcp_end_pose[1],
                                                                [0.0, 0.0, 0.0],
                                                                p.getQuaternionFromEuler([-3.14, 0.0, 0.0]))

        return world_tcp_end_worldoriented_pose

    def tcp_go_pose(self, target_pos, target_orien_q, tool_orien_e=None, print_value=False):
        """Class method that controls robot position by pose (i.e., position + orientation)
        target_pos (list): position
        target_orien_q (list): target orientation, in quaternions

        return the position to be given to the tcp looking to that object"""

        if(tool_orien_e == None):
            tool_orien_e = self.tool_orient_e

        world_object_position = target_pos
        world_object_orientation_q = target_orien_q

        # object position and TCP position are the same, the only difference is a rotation
        object_tcp_end_position = [0.0, 0.0, 0.0]
        object_tcp_end_orientation_q = p.getQuaternionFromEuler(tool_orien_e)

        # get the world_TCP pose (rotate)
        world_tcp_end_pose = p.multiplyTransforms(world_object_position, world_object_orientation_q,
                                                  object_tcp_end_position,
                                                  object_tcp_end_orientation_q)
        if print_value:
            print("\n", "world to tcp pose", world_tcp_end_pose[0], p.getEulerFromQuaternion(world_tcp_end_pose[1]))

        # from TCP to base, there is only a translation, the TCP offset
        tcp_end_base_position = [-1 * self.tcp_offset_pos[0], -1 * self.tcp_offset_pos[1], -1 * self.tcp_offset_pos[2]]
        tcp_end_base_orientation_q = p.getQuaternionFromEuler([0.0, 0.0, 0.0])

        # get the world TCP pose (translate)
        world_base_pose = p.multiplyTransforms(world_tcp_end_pose[0], world_tcp_end_pose[1], tcp_end_base_position,
                                               tcp_end_base_orientation_q)
        if print_value:
            print("\n", "world to base pose", world_base_pose[0], p.getEulerFromQuaternion(world_base_pose[1]))

        # from base to last link there is only one rotatiton
        base_lastlink_position = [0.0, 0.0, 0.0]
        base_lastlink_orientation_q = p.getQuaternionFromEuler([0.0,0.0,0.0])

        # get world's last link pose
        world_lastlink_pose = p.multiplyTransforms(world_base_pose[0], world_base_pose[1], base_lastlink_position,
                                                   base_lastlink_orientation_q)
        if print_value:
            print("\n", "world to lastlink pose", world_lastlink_pose[0],
                  p.getEulerFromQuaternion(world_lastlink_pose[1]))

        return world_lastlink_pose


    def get_object_position(self, objectID, modify_center=[0.0, 0.0, 0.0], orientation_correction_e=[-1.57, 0, 1.57]):
        self.objectIDpick = objectID
        object_link_info = p.getBasePositionAndOrientation(self.objectIDpick)
        object_position = object_link_info[0]  # refered to world
        object_orientation = object_link_info[1]  # refered to world
        print("\n", "world object pose", object_position, object_orientation)
        object_position_pick = list(object_position)  # refered to world
        object_position_pick = [object_position_pick[0] - modify_center[0], object_position_pick[1] - modify_center[1],
                                object_position_pick[2] - modify_center[2]]
        object_orientation_pick_e = p.getEulerFromQuaternion(object_orientation)
        object_orientation_pick_e = [object_orientation_pick_e[0] - orientation_correction_e[0],
                                     object_orientation_pick_e[1] - orientation_correction_e[1],
                                     object_orientation_pick_e[2] - orientation_correction_e[2]]
        object_orientation_pick_q = p.getQuaternionFromEuler(object_orientation_pick_e)
        return [object_position_pick, object_orientation_pick_q]

    def wait(self, time_wait):
        if self.visual_inspection:
            t = int(240 * time_wait)
        else:
            t = int(time_wait * 20)
        for i in range(t):
            self.step_simulation()


    def step_simulation(self):
        """Step simulation method"""
        p.stepSimulation()
        if self.visual_inspection:
            time.sleep(1.0 / 240.0)


    def create_empty_file(self,path):
        "Create a new file where you can write without losing data"
        aux_path = path [:-5] #copy all except the .urdf
        for  i in range(256):
            aux_path_2 = aux_path + "_" + str(i) + ".urdf"
            try:
                f = open(aux_path_2, "x")
                f.close
                return aux_path_2
                break
            except:
                print(aux_path_2 + " already exist" )
        return "error"

    def Copy_file(self,path2read,path2copy):
        readf = open(path2read,"r")
        writef = open(path2copy,"w")
        for line in readf:
            writef.write(line)


    def modify_elements_joint(self):
        "provide the options to modify the joints"
        joint_mod = ["damping","friction","lower", "upper", "effort", "velocity"]
        return joint_mod

    def modify_elements_link(self):
        "provide the options to modify the links"
        link_mod = ["mass","inertia"]
        return link_mod

    def modify_urdf(self,path2read,path2write,element_to_modify, value , link_or_joint_name = None):
        """
        To work with an specific name it's need it to be all together without space name=something >
        """


        #load the options
        joint_opt = self.modify_elements_joint()
        link_opt = self.modify_elements_link()

        #elements field
        inertial_elements = ["mass","inertia"]
        dynamics_elements = ["damping","friction"]
        limit_elements = ["lower", "upper", "effort", "velocity"]

        #1rst and 2nd element to search
        #Check if we have to search a joint or a link
        if (element_to_modify in joint_opt):
            opening_search_1 = "<joint"
            closing_search_1 = "</joint"
            if(link_or_joint_name == None):
                opening_search_2 = "name"
            else:
                opening_search_2 = "name=\""+str(link_or_joint_name)+"\""
            closing_search_2 = "</joint"

        elif((element_to_modify in link_opt)):
            opening_search_1 = "<link"
            closing_search_1 = "</link"
            if(link_or_joint_name == None):
                opening_search_2 = "name"
            else:
                opening_search_2 = "name=\""+str(link_or_joint_name)+"\""
                print(opening_search_2)
            closing_search_2 = "</link"
        else:
            print("Doesn't exist the field you are asking for, check modify_elements_link and modify_elements_joint ")
            return

        #3rd element to search
        if (element_to_modify in inertial_elements):
            opening_search_3 = "<inertial"
            closing_search_3 = "</inertial"
        elif((element_to_modify in dynamics_elements)):
            opening_search_3 = "<dynamics"
            closing_search_3 = "/>"
        elif((element_to_modify in limit_elements)):
            opening_search_3 = "<limit"
            closing_search_3 = "/>"
        else:
            print("Doesn't exist an element field, please add it to the function")
            return

        #4rd element to search and write
        if (element_to_modify == "mass"):
            opening_search_4 = "value"
            closing_search_4 = "/>"
            writevalue = " =  \"" + str(value) + "\" "

        elif(element_to_modify == "inertia"):
            opening_search_4 = "ixx"
            closing_search_4 = "/>"

            if(len(value) != 6):
                print("The given inertia is wrong please insert a list of the 6 elements of inertia")
                print("ixx, ixy, ixz, iyy, iyz, izz")
                return
            else:
                writevalue = " = \"" + str(value[0]) + "\" "  + \
                        " = \"" + str(value[1]) + "\" "  + \
                        " = \"" + str(value[2]) + "\" "  + \
                        " = \"" + str(value[3]) + "\" "  + \
                        " = \"" + str(value[4]) + "\" "  + \
                        " = \"" + str(value[5]) + "\" "
        elif(element_to_modify == "damping"):
            opening_search_4 = "damping"
            closing_search_4 = "\""
            writevalue = " = \"" + str(value)

        elif(element_to_modify == "friction"):
            opening_search_4 = "friction"
            closing_search_4 = "/>"
            writevalue = " = \"" + str(value) + "\" "

        elif(element_to_modify == "lower"):
            opening_search_4 = "lower"
            closing_search_4 = "\""
            writevalue = " = \"" + str(value)

        elif(element_to_modify == "upper"):
            opening_search_4 = "upper"
            closing_search_4 = "\""
            writevalue = " = \"" + str(value)

        elif(element_to_modify == "effort"):
            opening_search_4 = "effort"
            closing_search_4 = "\""
            writevalue = " = \"" + str(value)

        elif(element_to_modify == "velocity"):
            opening_search_4 = "velocity"
            closing_search_4 = "/>"
            writevalue = " = \"" + str(value)

        else:
            print("An error happend check this function")

        #Everything it's defined

        #Process of modification
        readf = open(path2read, "r")
        # for line in readf :
        #     print(line)
        writef = open(path2write, "w")
        print ("Overwriting " + path2write + " Reading " + path2read )

        auxtext = ""
        for line in readf :
            auxtext = auxtext + line
            if(opening_search_1 in auxtext):
                # print("I find the first")
                auxindex_open_1 = auxtext.find(opening_search_1)
                auxindex_open_2 = auxtext.find(opening_search_2, auxindex_open_1,)
                auxindex_close_1 = auxtext.find(closing_search_1, auxindex_open_1)

                #If it has found the opening and the closing do something else continue
                if((auxindex_open_2 != -1) and (auxindex_close_1 != -1)):
                    # print("I find the second")
                    #it's not the one I am searching so write until the close
                    if(auxindex_close_1 < auxindex_open_2):
                        #split the data to free memory
                        [textwrite,auxtext] = auxtext.split(closing_search_1,1)
                        textwrite = textwrite + closing_search_1
                        writef.write(textwrite)
                        textwrite = ""
                    else:
                        auxindex_open_3 = auxtext.find(opening_search_3,auxindex_open_2)
                        auxindex_close_2 = auxtext.find(closing_search_2,auxindex_open_1)

                        #If it has found the opening and the closing do something, else continue
                        if((auxindex_open_3 != -1) and (auxindex_close_2 != -1)):
                            # print("I find the third")
                            #it's not the one I am searching so write until the close
                            if(auxindex_close_2 < auxindex_open_3):
                                #split the data to free memory
                                [textwrite,auxtext] = auxtext.split(closing_search_2,1)
                                textwrite = textwrite + closing_search_2
                                writef.write(textwrite)
                                textwrite = ""
                            else:
                                auxindex_open_4 = auxtext.find(opening_search_4, auxindex_open_3)
                                auxindex_close_3 = auxtext.find(closing_search_3, auxindex_open_2)

                                #If it has found the opening and the closing do something, else continue
                                if((auxindex_open_4 != -1) and (auxindex_close_3 != -1)):
                                    # print("I find the fourth")
                                    #it's not the one I am searching so write until the close
                                    if(auxindex_close_3 < auxindex_open_4):
                                        #split the data to free memory
                                        [textwrite,auxtext] = auxtext.split(closing_search_3,1)
                                        textwrite = textwrite + closing_search_3
                                        writef.write(textwrite)
                                        textwrite = ""
                                    else:
                                        auxindex_close_4 = auxtext.find(closing_search_4, auxindex_open_4)
                                        #If it has found the opening and the closing do something, else continue
                                        if((auxindex_open_4 != -1) and (auxindex_close_4 != -1)):
                                            print("I find all")
                                            [textwrite, auxtext] = auxtext.split(opening_search_4,1)
                                            textwrite = textwrite + opening_search_4
                                            writef.write(textwrite)
                                            writef.write(writevalue)
                                            [omit,textwrite] = auxtext.split(closing_search_4,1)
                                            textwrite = closing_search_4 + textwrite
                                            writef.write(textwrite)
                                            #clean to read from that point
                                            auxtext = ""
        writef.write(auxtext)
        readf.close()
        writef.close()
        print("Files closed")



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
