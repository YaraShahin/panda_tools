#!/usr/bin/env python

import tf
import sys
import rospy
import actionlib
import numpy as np
import moveit_msgs.msg
import moveit_commander
from copy import deepcopy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import MoveAction
from franka_gripper.msg import HomingAction
from franka_gripper.msg import MoveActionGoal     # imported to populate goal with the correct syntax
from tf.transformations import quaternion_matrix, quaternion_from_matrix


class Robot(object):

    def __init__(self):
        super(Robot, self).__init__()
        self.robot_state = moveit_commander.RobotCommander()
        self.setup_planner()
        self.listener = tf.TransformListener()
        self.scene = moveit_commander.PlanningSceneInterface()
        # self.grippers = GripperClient()
        self.JOINT_BASE = 0
        self.JOINT_WRIST = 6
        self.ee_to_finger = 0.13
        self.replan = False
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        # self.pub_pose = rospy.Publisher('/our/test/pose_stamped_pos', PoseStamped, queue_size=1)  # TODO: delete

    def setup_planner(self):
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group.set_end_effector_link("panda_hand")    # planning wrt to panda_hand or link8
        self.group.set_max_velocity_scaling_factor(0.20)  # scaling down velocity
        self.group.set_max_acceleration_scaling_factor(0.05)  # scaling down velocity
        self.group.allow_replanning(True)
        self.group.set_num_planning_attempts(10)
        self.group.set_goal_position_tolerance(0.0005)
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.set_planning_time(5)
        self.group.set_planner_id("FMTkConfigDefault")
        rospy.sleep(2)
        print("Ready to go")

    def rotate_joint(self, joint_id, angle):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[joint_id] = angle
        print("rotating joint ....")
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def go_home(self):
        print("Going To Home Location")
        p = PoseStamped()
        p.header.frame_id = '/panda_link0'
        p.pose.position.x = 0.45
        p.pose.position.y = -0.25
        p.pose.position.z = 0.45    #was 0.55
        p.pose.orientation.x = 1
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 0
        target = self.group.set_pose_target(p)

        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # display_trajectory.trajectory_start = robot.get_current_state()
        # display_trajectory.trajectory.append(plan)
        # self.display_trajectory_publisher.publish(display_trajectory)

        self.group.go(target, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def go_box(self):
        print("Going To Box Location")
        p = PoseStamped()
        p.header.frame_id = '/panda_link0'
        p.pose.position.x = 0.484
        p.pose.position.y = 0.333
        p.pose.position.z = 0.340
        p.pose.orientation.x = 1
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 0
        target = self.group.set_pose_target(p)
        self.group.go(target, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

class GripperClient():

    def __init__(self):

        self.close_percentage = 0.65     #was 0.5
        self.grasp_clearence = 0.03    # was 0.02    
        self.finger_offset = 0.01
        self.homing_action()

    def homing_action(self):
        client = actionlib.SimpleActionClient("/franka_gripper/homing", HomingAction)
        client.wait_for_server()
        client.send_goal(True)
        homing_done = client.wait_for_result()
        print("#### homing gripper ...")
        return homing_done

    def move_action(self, w, s):
        client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        client.wait_for_server()
        goal = MoveActionGoal
        goal.width = w
        goal.speed = s
        client.send_goal(goal)
        print("#### moving gripper to set width=", goal.width, "set speed=", goal.speed)
        move_done = client.wait_for_result()

        return move_done

    def close(self, diameter):
        print("OPEN DIAMETER", diameter)
        offset_maths = self.close_percentage * (diameter - self.finger_offset)
        print("OPEN DIAMETER2", offset_maths)
        self.move_action(offset_maths, 0.02)  # width, speed

    def open(self, diameter):
        print("CLOSE DIAMETER", diameter)
        offset_maths = (diameter - self.finger_offset + self.grasp_clearence)
        print("CLOSE DIAMETER2", offset_maths)
        self.move_action(offset_maths, 0.02)  # width, speed


def main():
    robot = Robot()
    robot.go_home()
    robot.rotate_joint(1,0.3)
    robot.go_box()
    rospy.spin()

    # grippers = GripperClient()
    # grippers.open(0.2)
    # grippers.close(0.2)

if __name__ == '__main__':
    rospy.init_node('Move_Robot')
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        main()
    except Exception as e:
        print(e)
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
