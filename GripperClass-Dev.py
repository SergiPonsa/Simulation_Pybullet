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
from constants import TCP_INIT_POSE


# ------------------------------------------------------------------------------------------------------

class Gripper():
    """Generic Robot class"""

    def __init__(self,
                urdf_root = pybullet_data.getDataPath(),
                root = "",

                tool_class = "",
                tool_urdf = "",
                tool_control_joints = [],
                tool_mimic_joints_name = [],
                tool_mimic_joints_master = [],
                tool_mimic_multiplier = [],

                tcp_offset_pos = [0.0, 0.0, 0.0],
                tcp_offset_orien_e = [0.0, 0.0, 0.0]):

        """Initialization function

        urdf_root (str): root for URDF objects
        root (str): root directory for local files

        tool_urdf (str): name and location of tool URDF file
        tool_control_joints (list of str): name tool control joints in order to control them
        tool_mimic_joints_name (list of str): name tool joints which follow control joints in order to control them
        tool_mimic_joints_master (list of str): name joints which master each mimic joint
        tool_mimic_multiplier (list of doubles): value of increase of the mimic respect the by increase of the master

        tcp_offset_pos (list of doubles | x,y,z in m): position offset referent to the robot_conection_to_tool_name
        tcp_offset_orien (list of doubles | rx,ry,rz in rad RPY): position offset referent to the robot_conection_to_tool_name

        """

        self.urdf_root = urdf_root
        self.robot_urdf = os.path.join(root, robotgripper_urdf)
        self.gripper_fingers_control_name = gripper_fingers_control_name
        self.last_robot_joint_name = last_robot_joint_name
        self.urdf_error_correct_orien_e = urdf_error_correct_orien_e
        self.tcp_offset_pos = tcp_offset_pos
        self.tcp_offset_orien_e = tcp_offset_orien_e
        self.robot_control_joints = robot_control_joints
        self.gripper_main_control_joint = gripper_main_control_joint
        self.mimic_joint_name = mimic_joint_name
        self.mimic_multiplier = mimic_multiplier
        self.home = home
        self.VISUAL_INSPECTION = visual_inspection

        # initialize variables
        self.robot_control_joints_index = [0, 0, 0, 0, 0, 0]  # to avoid errors
        self.opening_length = 0.085  # start with the gripper open

        # launch robot
        robotStartPos = [0, 0, 0]  # don't modify!
        robotStartOrien = p.getQuaternionFromEuler([0, 0, 0])  # don't modify!
        self.robotID = p.loadURDF(os.path.join(root, "urdf/ur5_rf85.urdf"), robotStartPos, robotStartOrien,
                                  flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
        # robot data structure
        jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        jointInfo = namedtuple("jointInfo",
                               ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity"])
        self.joints = AttrDict()
        self.joint_names = []
        # get robot data from urdf
        numJoints = p.getNumJoints(self.robotID)
        for i in range(numJoints):
            info = p.getJointInfo(self.robotID, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            self.joint_names.append(jointName)
            jointType = jointTypeList[info[2]]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            singleInfo = jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce,
                                   jointMaxVelocity)
            self.joints[singleInfo.name] = singleInfo
            # get data from control joints
            for k in range(len(self.robot_control_joints)):
                if (jointName == self.robot_control_joints[k]):
                    self.robot_control_joints_index[k] = i
            if jointName == self.last_robot_joint_name:
                self.last_robot_joint_index = i
        self.joint_param_value=home

        # parameters for the nullspace
        ll = []
        ul = []
        jr = []
        rp = []
        # get robot data from urdf
        numJoints = p.getNumJoints(self.robotID)
        for i in range(numJoints):
            ll.append(self.joints[self.joint_names[i]].lowerLimit)
            ul.append(self.joints[self.joint_names[i]].upperLimit)
            jr.append(
                abs(self.joints[self.joint_names[i]].lowerLimit) + abs(self.joints[self.joint_names[i]].upperLimit))
            rp.append(0)

        # Tell that the solution has to be near to the home position
        for i in range(len(self.home)):
            rp[i] = self.home[i]
        self.lower_limit=ll
        self.upper_limit=ul
        self.joint_range=jr
        self.resting_pose=rp

    def move_joints(self, joint_param_value=None, wait=True):
        """Class method to control robot position by passing joint angles
        joint_param_value (list): joint angles aimed at"""
        if joint_param_value is None:
            joint_param_value = self.joint_param_value

        reached = False
        counter = 0
        while not reached:
            counter += 1
            # change joint angles
            for i in range(len(self.robot_control_joints)):
                p.setJointMotorControl2(self.robotID, self.joints[self.robot_control_joints[i]].id,
                                        p.POSITION_CONTROL, targetPosition=joint_param_value[i],
                                        force=self.joints[self.robot_control_joints[i]].maxForce,
                                        maxVelocity=self.joints[self.robot_control_joints[i]].maxVelocity)
            if wait:
                # make step simulation
                self.step_simulation()
                if self.VISUAL_INSPECTION:
                    time.sleep(1. / 240.)  # to make the simulation of the GUI in real time
                # check position reached
                for i in range(len(self.robot_control_joints)):
                    jointstate_aux = p.getJointState(self.robotID, self.robot_control_joints_index[i])
                    if i == 0:
                        jointstatepos = [jointstate_aux[0]]
                        jointdiff = abs(jointstatepos[i] - self.home[i])
                    else:
                        jointstatepos.append(jointstate_aux[0])
                        jointdiff = jointdiff + abs(jointstatepos[i] - self.home[i])
                if (jointdiff <= 10 ** -2) or (counter > 256):
                    reached = True
            else:
                reached = True

    def get_actual_joints_angle(self):
        for i in range(len(self.robot_control_joints)):
            jointstate_aux = p.getJointState(self.robotID, self.robot_control_joints_index[i])
            if i == 0:
                jointstatepos = [jointstate_aux[0]]
            else:
                jointstatepos.append(jointstate_aux[0])
        return jointstatepos

    def get_actual_joints_torque(self):
        for i in range(len(self.robot_control_joints)):
            jointstate_aux = p.getJointState(self.robotID, self.robot_control_joints_index[i])
            if i == 0:
                jointstatetorque = [
                    jointstate_aux[3]]  # these indexes are: [0]Position, [1]Speed, [2]Reactive str, [3]Torque
            else:
                jointstatetorque.append(jointstate_aux[3])
        return jointstatetorque

    def get_actual_joints_full(self):
        jointsdata=[0]*3*len(self.robot_control_joints)
        for i in range(len(self.robot_control_joints)):
            jointstate_aux = p.getJointState(self.robotID, self.robot_control_joints_index[i])
            for j,k in enumerate([0,1,3]):
                jointsdata[i*3+j] = jointstate_aux[k]
        return jointsdata

    def move_cartesian(self, pose, nullspace=True):
        """Class method to control the robot position by passing space coordinates and orientation and working out the correspondence to
        joint angles to call 'move_joints'
        pose (list): pose, i.e., position + orientation (as roll, pitch, yaw)"""

        # Equations to ensure it's inside the reachable area (NOT NEEDED!)
        '''# maixmum and minimum are blocked in the debug, no need to check
        if(pos_param_value[i]<pos_param_min_value[i]):
            pos_param_value[i]=pos_param_min_value[i]
        elif (pos_param_value[i]>pos_param_max_value[i]):
            pos_param_value[i]=pos_param_max_value[i]
        # check y is inside the rechable disk equation [x,y], h & k center
        # (x−h)^2+(y−k)^2=r^2
        try:
            if ( ( (pos_param_value[0]-0.0)**2 + (pos_param_value[1]-0.0)**2 ) > ( robot_reachability_radius**2 ) ):
                if(pos_param_value[1] > 0):
                    pos_param_value[1] = (robot_reachability_radius**2 - pos_param_value[0]**2)**0.5
                else:
                    pos_param_value[1] = -(robot_reachability_radius**2 - pos_param_value[0]**2)**0.5
        except: # If can't compute, means x² > radius so
            if(pos_param_value[1] > 0):
                pos_param_value[0] =robot_reachability_radius
            else:
                pos_param_value[0] =-robot_reachability_radius
            pos_param_value[1] =0.0
        # check z is inside the rechable sphere equation
        # (x−h)^2+(y−k)^2+(z−l)^2=r^2
        try:
            if ( ( (pos_param_value[0]-0.0)**2 +(pos_param_value[1]-0.0)**2 + (pos_param_value[2]-0.0)**2 ) > ( robot_reachability_radius**2 ) ):
                if(pos_param_value[2] > 0):
                    pos_param_value[2] = ((robot_reachability_radius**2 - pos_param_value[1]**2 - pos_param_value[0]**2 )**0.5)
                else:
                    pos_param_value[2] = -((robot_reachability_radius**2 - pos_param_value[1]**2 - pos_param_value[0]**2 )**0.5)
        except:                      # if can't compute, means x² +y²  > radius so
            pos_param_value[2] =0.0
        # pose[3]=urdf_error_correct_orien_e[0] ???'''

        if (nullspace == True):

            inv_result = p.calculateInverseKinematics(self.robotID, self.last_robot_joint_index, pose[0], pose[1],
                                                      maxNumIterations=500,
                                                      lowerLimits=self.lower_limit,
                                                      upperLimits=self.upper_limit,
                                                      jointRanges=self.joint_range,
                                                      restPoses=self.resting_pose)
        else:
            inv_result = p.calculateInverseKinematics(self.robotID, self.last_robot_joint_index, pose[0], pose[1],
                                                      maxNumIterations=500)

        self.joint_param_value = [inv_result[0], inv_result[1], inv_result[2], inv_result[3], inv_result[4],
                             inv_result[5]]

        # perform control action with 'joint_param_value'
        self.move_joints(wait=False)

    def move_home(self):
        """Class method that sends robot to 'home' position"""
        self.move_joints(self.home)

    def get_actual_tcp_pose(self, print_value=False):
        last_robot_link_info = p.getLinkState(self.robotID, self.last_robot_joint_index)
        world_last_robot_link_position = last_robot_link_info[0]
        world_last_robot_link_orientation_q = last_robot_link_info[1]
        world_last_robot_link_orientation_e = p.getEulerFromQuaternion(last_robot_link_info[1])
        if print_value:
            print("world's last robot joint position", world_last_robot_link_position,
                  world_last_robot_link_orientation_e)
        last_robot_link_tcp_base_position = [0, 0, 0]
        last_robot_link_tcp_base_orientation_e = [self.urdf_error_correct_orien_e[0] + self.tcp_offset_orien_e[0],
                                                  self.urdf_error_correct_orien_e[1] + self.tcp_offset_orien_e[1],
                                                  self.urdf_error_correct_orien_e[2] + self.tcp_offset_orien_e[2]]
        last_robot_link_tcp_base_orientation_q = p.getQuaternionFromEuler(last_robot_link_tcp_base_orientation_e)

        # transform from TCP base to world
        world_tcp_base_pose = p.multiplyTransforms(world_last_robot_link_position, world_last_robot_link_orientation_q,
                                                   last_robot_link_tcp_base_position,
                                                   last_robot_link_tcp_base_orientation_q)
        tcp_base_tcp_end_position = self.tcp_offset_pos
        tcp_base_tcp_end_orientation_q = p.getQuaternionFromEuler([0, 0, 0])

        # transform from TCP end to world
        world_tcp_end_pose = p.multiplyTransforms(world_tcp_base_pose[0], world_tcp_base_pose[1],
                                                  tcp_base_tcp_end_position, tcp_base_tcp_end_orientation_q)

        if print_value:
            print("\n", "world to tcp end position", world_tcp_end_pose[0],
                  p.getEulerFromQuaternion(world_tcp_end_pose[1]))
        return world_tcp_end_pose

    def get_actual_tcp_pose_world_oriented(self, print_value=False):
        world_tcp_end_pose = self.get_actual_tcp_pose()
        # I want world_tcp_end_worldoriented_pose = world_tcp_end_pose *tcp_end_tcp_end_worldoriented_pose
        # object position and TCP position are the same, the only difference is a rotation

        world_tcp_end_worldoriented_pose = p.multiplyTransforms(world_tcp_end_pose[0], world_tcp_end_pose[1],
                                                                [0.0, 0.0, 0.0],
                                                                p.getQuaternionFromEuler([-3.14, 0.0, 0.0]))

        return world_tcp_end_worldoriented_pose

    def tcp_go_pose(self, target_pos, target_orien_q, gripper_object_orien_e=[3.14, 0.0, 0.0], print_value=False):
        """Class method that controls robot position by pose (i.e., position + orientation)
        target_pos (list): position
        target_orien_q (list): target orientation, in quaternions"""
        world_object_position = target_pos
        world_object_orientation_q = target_orien_q

        # object position and TCP position are the same, the only difference is a rotation
        object_tcp_end_position = [0.0, 0.0, 0.0]
        object_tcp_end_orientation_q = p.getQuaternionFromEuler(gripper_object_orien_e)

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
        base_lastlink_orientation_q = p.getQuaternionFromEuler(self.urdf_error_correct_orien_e)

        # get world's last link pose
        world_lastlink_pose = p.multiplyTransforms(world_base_pose[0], world_base_pose[1], base_lastlink_position,
                                                   base_lastlink_orientation_q)
        if print_value:
            print("\n", "world to lastlink pose", world_lastlink_pose[0],
                  p.getEulerFromQuaternion(world_lastlink_pose[1]))

        return world_lastlink_pose

    def control_gripper(self, gripper_opening_length): # Refactor this!!
        self.opening_length = gripper_opening_length
        if gripper_opening_length > 0.085:
            gripper_opening_length = 0.085
            #print("length too wide, open only 0.085")
        if gripper_opening_length < 0.0:
            gripper_opening_length = 0.0
            #print("length negative, open only 0.0")

        # do the grasping
        gripper_opening_angle = 0.715 - math.asin((gripper_opening_length - 0.010) / 0.1143)  # angle calculation

        # gripper_opening_angle_actual = p.getJointState(self.robotID,self.joints[self.gripper_main_control_joint].id)
        # control finger tips
        for k in range(len(self.gripper_fingers_control_name)):
            joint = self.joints[self.gripper_fingers_control_name[k]]
            if ((self.gripper_fingers_control_name[k] == "robotiq_85_right_finger_tip_z") or (
                    self.gripper_fingers_control_name[k] == "robotiq_85_left_finger_tip_z")):
                # param = 0.1143 * math.cos(gripper_opening_angle-0.715)
                param = 0.1143 * math.cos(gripper_opening_angle - 0.715) + 0.01
            elif (self.gripper_fingers_control_name[k] == "robotiq_85_right_finger_tip_y"):
                # param = -1 * ((self.opening_length/2)+0.01)
                param = 0.009 + (self.opening_length / 2)
            elif (self.gripper_fingers_control_name[k] == "robotiq_85_left_finger_tip_y"):
                # param = ((self.opening_length/2)+0.01)
                param = -0.009 - (self.opening_length / 2)
            p.setJointMotorControl2(self.robotID, joint.id, p.POSITION_CONTROL,
                                    targetPosition=param,
                                    force=joint.maxForce,
                                    maxVelocity=joint.maxVelocity,
                                    positionGain=1.5,
                                    velocityGain=1.1)

        p.setJointMotorControl2(self.robotID,
                                self.joints[self.gripper_main_control_joint].id,
                                p.POSITION_CONTROL,
                                targetPosition=gripper_opening_angle,
                                force=self.joints[self.gripper_main_control_joint].maxForce,
                                maxVelocity=self.joints[self.gripper_main_control_joint].maxVelocity,
                                positionGain=1.5,  # gain position error modification
                                velocityGain=1.1)  # gain velocity error modification

        for i in range(len(self.mimic_joint_name)):
            joint = self.joints[self.mimic_joint_name[i]]
            p.setJointMotorControl2(self.robotID, joint.id, p.POSITION_CONTROL,
                                    targetPosition=gripper_opening_angle * self.mimic_multiplier[i],
                                    force=joint.maxForce,
                                    maxVelocity=joint.maxVelocity,
                                    positionGain=1.5,
                                    velocityGain=1.1)

        # simulate to move the gripper
        p.stepSimulation()
        if self.VISUAL_INSPECTION:
            time.sleep(1.0 / 240.0)

        # update the joints to be real, get the real length an compute the real z and gripper positions
        opening_length_sensor = self.gripper_length()

        try:
            gripper_opening_angle = 0.715 - math.asin(
                (opening_length_sensor - 0.010) / 0.1143)  # real angle calculation
        except ValueError:
            gripper_opening_angle = 0.715

        # move joints to real state to make the photo
        for k in range(len(self.gripper_fingers_control_name)):
            joint = self.joints[self.gripper_fingers_control_name[k]]
            if ((self.gripper_fingers_control_name[k] == "robotiq_85_right_finger_tip_z") or (
                    self.gripper_fingers_control_name[k] == "robotiq_85_left_finger_tip_z")):
                # param = 0.1143 * math.cos(gripper_opening_angle-0.715)
                param = 0.1143 * math.cos(gripper_opening_angle - 0.715) + 0.01
            elif (self.gripper_fingers_control_name[k] == "robotiq_85_right_finger_tip_y"):
                # param = -1 * ((self.opening_length/2)+0.01)
                param = 0.009 + (self.opening_length / 2)
            elif (self.gripper_fingers_control_name[k] == "robotiq_85_left_finger_tip_y"):
                # param = ((self.opening_length/2)+0.01)
                param = -0.009 - (self.opening_length / 2)
            p.setJointMotorControl2(self.robotID, joint.id, p.POSITION_CONTROL,
                                    targetPosition=param,
                                    force=joint.maxForce,
                                    maxVelocity=joint.maxVelocity,
                                    positionGain=1.5,
                                    velocityGain=1.1)

        p.setJointMotorControl2(self.robotID,
                                self.joints[self.gripper_main_control_joint].id,
                                p.POSITION_CONTROL,
                                targetPosition=gripper_opening_angle,
                                force=self.joints[self.gripper_main_control_joint].maxForce,
                                maxVelocity=self.joints[self.gripper_main_control_joint].maxVelocity,
                                positionGain=1.5,  # gain position error modification
                                velocityGain=1.1)  # gain velocity error modification

        for i in range(len(self.mimic_joint_name)):
            joint = self.joints[self.mimic_joint_name[i]]
            p.setJointMotorControl2(self.robotID, joint.id, p.POSITION_CONTROL,
                                    targetPosition=gripper_opening_angle * self.mimic_multiplier[i],
                                    force=joint.maxForce,
                                    maxVelocity=joint.maxVelocity,
                                    positionGain=1.5,
                                    velocityGain=1.1)

    def gripper_length(self):
        """Class method that calculates the distance between the fingers of the gripper"""
        right_finger_tip_position = p.getJointState(self.robotID, self.joints["robotiq_85_right_finger_tip_y"].id)
        left_finger_tip_position = p.getJointState(self.robotID, self.joints["robotiq_85_left_finger_tip_y"].id)
        gripper_opening_length = abs(right_finger_tip_position[0]) + abs(left_finger_tip_position[0]) - (
            0.009) * 2  # minus the gap allowed
        return gripper_opening_length

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
        if self.VISUAL_INSPECTION:
            t = int(240 * time_wait)
        else:
            t = int(time_wait * 20)
        for i in range(t):
            self.step_simulation()
            if self.VISUAL_INSPECTION:
                time.sleep(1.0 / 240.0)

    def step_simulation(self):
        """Step simulation method"""
        p.stepSimulation()
        if self.VISUAL_INSPECTION:
            time.sleep(1.0 / 240.0)


# ------------------------------------------------------------------------------------------------------

if __name__ == '__main__':

    p.connect(p.GUI)
    p.setGravity(0.0, 0.0, -9.81)

    # create robot instance with default configuration
    robot = ur5_rf85()
    robot.move_home()
    try:
        while True:
            robot.step_simulation()
    except KeyboardInterrupt:
        p.disconnect()

# ------------------------------------------------------------------------------------------------------
