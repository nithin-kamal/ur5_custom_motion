import sys

import rospy
import moveit_commander
from geometry_msgs.msg import Pose


rospy.wait_for_service('/move_group/trajectory_execution/set_parameters')

class BotMoveGroup:
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        
    def interface_primary_movegroup(self):
        self.robot, self.planning_scene, self.move_group = self._initialize_planner()
        
    def _initialize_planner(self):
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        return robot, scene, move_group
    
    def execute_joint_trajectory(self, target_joints, velocity = 3.14, acceleration = 1.0):
        self.move_group.set_start_state_to_current_state()
        vel_scaling_factor = min(1, velocity % 3.14)
        acc_scaling_factor = min(1, acceleration % 1.0)
        rospy.loginfo("Velocity scaling factor: %f", vel_scaling_factor)
        rospy.loginfo("Acceleration scaling factor: %f", acc_scaling_factor)    
        # self.move_group.set_max_velocity_scaling_factor(vel_scaling_factor)
        # self.move_group.set_max_acceleration_scaling_factor(acc_scaling_factor)
        self.move_group.set_joint_value_target(target_joints)
        return self.move_group.go()
    
    def execute_pose_trajectory(self, poseList, velocity = 3.14, acceleration = 1.0):
        self.move_group.set_end_effector_link("tool0")
        self.move_group.set_start_state_to_current_state()
        target_pose = self.list_to_pose(poseList)
        rospy.loginfo(target_pose)
        vel_scaling_factor = min(1, velocity % 3.14)
        acc_scaling_factor = min(1, acceleration % 1.0)    
        # self.move_group.set_max_velocity_scaling_factor(vel_scaling_factor)
        # self.move_group.set_max_acceleration_scaling_factor(acc_scaling_factor)
        self.move_group.set_pose_target(target_pose)
        output = self.move_group.go()
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return output
    
    def list_to_pose(self, poseList):
        target_pose = Pose()
        target_pose.position.x = poseList[0]
        target_pose.position.y = poseList[1]
        target_pose.position.z = poseList[2]
        target_pose.orientation.w = poseList[3]
        return target_pose