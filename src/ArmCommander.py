#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
import getch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import argparse

def wait_for_keypress():
    print("Press any key to continue...")
    getch.getch()
    print("Continuing...")

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface(object):
  HOME_POSE = geometry_msgs.msg.Pose()
  HOME_POSE.position.x, HOME_POSE.position.y, HOME_POSE.position.z= 0.37896, 0.013066, 0.21587
  HOME_POSE.orientation.x, HOME_POSE.orientation.y, HOME_POSE.orientation.z, HOME_POSE.orientation.w = -0.93856, 0.3445, -0.012249, 0.01666
  
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
    debugging = rospy.get_param('~debugging', True)

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    rospy.Subscriber("/target_pose", geometry_msgs.msg.Pose, self.robot_commander_callback)
    
    ## Create a `TargetPose`_ ROS publisher which is used to display
    ## poses in Rviz:
    rviz_display_pose_publisher = rospy.Publisher('/rviz_display_pose', 
                                            geometry_msgs.msg.PoseStamped, 
                                            latch=True, 
                                            queue_size=20)
    
    current_ee_pose_publisher = rospy.Publisher('/current_ee_pose', 
                                            geometry_msgs.msg.Pose, 
                                            queue_size=20)

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    rospy.loginfo("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    rospy.loginfo("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    rospy.loginfo("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    rospy.loginfo("============ Printing robot state")
    rospy.loginfo(robot.get_current_state())
    rospy.loginfo("")

    # Misc variables
    self.robot = robot
    self.debugging = debugging
    self.scene = scene
    self.move_group = move_group
    self.current_ee_pose_publisher = current_ee_pose_publisher
    self.rviz_display_pose_publisher = rviz_display_pose_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    
  def robot_commander_callback(self, msg):
    pose = msg.data
    rospy.loginfo("Received message: %s", msg.data)

    self.go_to_pose_goal(pose)
    
  def get_current_pose(self):
      current_pose = self.move_group.get_current_pose().pose
      return current_pose
    
  def display_pose(self, pose):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = self.planning_frame

    # Define the pose
    pose_msg = PoseStamped()
    pose_msg.header = header
    pose_msg.pose=pose
    self.rviz_display_pose_publisher.publish(pose_msg)

  def go_to_pose_goal(self, pose):
    """
    Panda End Effector go to pose
    debugging mode waits for ur approval on the computed trajectory
    """
    self.display_pose(pose)
    self.move_group.set_pose_target(pose)
    rospy.loginfo("============ Going to specified Target Pose:", pose)
    
    # make the plan and display the trajectory in RViz
    plan_success, trajectory, planning_time, error_code = self.move_group.plan(pose)
    
    if (self.debugging):
      wait_for_keypress()   #wait for planned trajectory human validation
    
    if trajectory:    # If planning succeeds, Execute the trajectory
      self.move_group.execute(trajectory)
    else:
      rospy.logerr("Failed to plan trajectory!")
    
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    
    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    
    if (all_close(pose, current_pose, 0.01)):
      rospy.loginfo("============ Movement Completed Successfully ===========")
      return 1
    else:
      rospy.loginfo("============ Movement Failed ===========")
      return 0
    
  def go_home(self):
    rospy.loginfo("============ Going to Home Position ===========")
    self.go_to_pose_goal(self.HOME_POSE, debugging=False)
    
def main():
  try:
    RobotCommander = MoveGroupPythonInteface()
    RobotCommander.go_home()
    rospy.loginfo("Robot Commander Ready to recieve pose messages!")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      pose_msg = RobotCommander.get_current_pose()
      RobotCommander.current_ee_pose_publisher.publish(pose_msg)
      rospy.loginfo("Published message: %s" % pose_msg)
      rate.sleep()
    rospy.spin()
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()