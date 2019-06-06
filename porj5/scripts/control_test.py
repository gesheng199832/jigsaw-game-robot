#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

class my_panda():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")
        self.display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory,
                                            queue_size=10)
                                        
    def move_arm(self, pose_target):
        '''
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = 0.92898
        pose_target.orientation.y = 0.3698
        pose_target.orientation.z = -0.00184
        pose_target.orientation.w = 0.01388
        pose_target.position.x = 0.4638
        pose_target.position.y = 0.04866
        pose_target.position.z = 0.0899
        '''
        self.arm_group.set_pose_target(pose_target)
        plan = self.arm_group.plan()
        
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        #print("go")
        #time.sleep(2)
        self.arm_group.go(wait=True)
        self.arm_group.clear_pose_targets()
    
    def close_gripper(self):
        try:
            self.hand_group.set_joint_value_target([0.001, 0.001])
        except:
            pass
        plan = self.hand_group.plan()
        '''
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory);
        '''
        self.hand_group.go(wait=True)
        self.hand_group.clear_pose_targets()
        
    def open_gripper(self):
        try:
            self.hand_group.set_joint_value_target([0.04, 0.04])
        except:
            pass
        plan = self.hand_group.plan()
        '''
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory);
        '''
        self.hand_group.go(wait=True)
        self.hand_group.clear_pose_targets()
    def set_goal(self, xyz, ori=[0.92898, 0.3698, -0.00184, 0.01388]):
    
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = xyz[0]
        pose_target.position.y = xyz[1]
        pose_target.position.z = xyz[2]
        pose_target.orientation.x = ori[0]
        pose_target.orientation.y = ori[1]
        pose_target.orientation.z = ori[2]
        pose_target.orientation.w = ori[3]
        return pose_target


