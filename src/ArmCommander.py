#!/usr/bin/env python

##########################################
#           Libs
##########################################
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
import getch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
import math

##########################################
#           Utils
##########################################
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

##########################################
#           Core
##########################################
class ArmCommander(object):
  def __init__(self):
    super(ArmCommander, self).__init__()
    self.debugging = rospy.get_param('~debugging', True)

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_commander', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    self.robot = moveit_commander.RobotCommander()
    self.setup_planner()
    self.log_robot_info()
    
    self.rviz_display_pose_publisher = rospy.Publisher('/rviz_display_pose', geometry_msgs.msg.PoseStamped, latch=True, queue_size=20)    #Target Pose
    self.current_ee_pose_publisher = rospy.Publisher('/current_ee_pose', geometry_msgs.msg.Pose, queue_size=20)
    self.stationary_status_publisher = rospy.Publisher('/stationary_status', Bool, queue_size=20)

    # To command the arm to go to a target ee pose, just publish a msg to this topic
    rospy.Subscriber("/target_ee_pose", geometry_msgs.msg.Pose, self.arm_commander_callback)
    
    self.old_pose = self.get_current_pose()
    
    #self.go_home()
    rospy.sleep(2)
    rospy.loginfo("Arm Commander Ready to recieve pose messages!")

  def log_robot_info(self):
    rospy.loginfo("============ Planning frame: %s" % self.planning_frame)
    rospy.loginfo("============ End effector link: %s" % self.eef_link)      
    rospy.loginfo("============ Available Planning Groups: %s" % self.robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    rospy.loginfo("============ Printing robot state")
    rospy.loginfo(self.robot.get_current_state())
    rospy.loginfo("")
    
    rospy.loginfo("Published current pose message: %s \n" % self.get_current_pose())  
    
  def setup_planner(self):
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## This interface can be used to plan and execute motions:
    self.group_name = rospy.get_param('/ArmCommander/group_name', "panda_arm")
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
    self.move_group.set_end_effector_link(rospy.get_param('/ArmCommander/ee_link', "panda_hand"))    # planning wrt to panda_hand or link8
    self.move_group.set_max_velocity_scaling_factor(rospy.get_param('/ArmCommander/max_vel',0.20))  # scaling down velocity
    self.move_group.set_max_acceleration_scaling_factor(rospy.get_param('/ArmCommander/max_accel',0.05))  # scaling down velocity
    self.move_group.allow_replanning(rospy.get_param('/ArmCommander/allow_replanning',True))
    self.move_group.set_num_planning_attempts(rospy.get_param('/ArmCommander/max_num_attempts',10))
    self.move_group.set_goal_position_tolerance(rospy.get_param('/ArmCommander/pos_tolerance',0.0005))
    self.move_group.set_goal_orientation_tolerance(rospy.get_param('/ArmCommander/orientation_tolerance',0.01))
    self.move_group.set_planning_time(rospy.get_param('/ArmCommander/planning_time',5))
    self.move_group.set_planner_id(rospy.get_param('/ArmCommander/planner_id',"FMTkConfigDefault"))
      
    self.planning_frame = self.move_group.get_planning_frame()  # reference frame for this robot
    self.eef_link = self.move_group.get_end_effector_link()
    self.group_names = self.robot.get_group_names()
    
  def go_home(self):
    rospy.loginfo("============ Going to Home Position ===========")
    joint_goal = rospy.get_param('/JOINT_STATES/HOME')
    self.go_to_joint_state(joint_goal)
    
  def arm_commander_callback(self, pose):
    rospy.loginfo("Received target pose message: %s", pose)

    self.go_to_pose_goal(pose, self.debugging)
    
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

  def go_to_pose_goal(self, pose, debugging = True):
    """
    Panda End Effector go to pose
    debugging mode waits for ur approval on the computed trajectory
    """
    self.display_pose(pose)
    self.move_group.set_pose_target(pose)
    rospy.loginfo("============ Going to specified Target Pose: %s" % pose)
    
    # make the plan and display the trajectory in RViz
    plan_success, trajectory, planning_time, error_code = self.move_group.plan(pose)
    
    if trajectory:    # If planning succeeds, Execute the trajectory
      if (debugging):
        wait_for_keypress()   #wait for planned trajectory human validation
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
    
  def go_to_joint_state(self, joint_goal):
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  
  def is_stationary(self):
    current_pose = self.get_current_pose()
    is_stationary_msg = Bool()
    if (all_close(current_pose, self.old_pose, 0.01)):
      is_stationary_msg.data = True
    else:
      is_stationary_msg.data = False
    self.old_pose = current_pose
    return is_stationary_msg
    

##########################################
#           Main
##########################################
def main():
  try:
    arm_commander = ArmCommander()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
      pose_msg = arm_commander.get_current_pose()
      arm_commander.current_ee_pose_publisher.publish(pose_msg)
      
      is_stationary_msg = arm_commander.is_stationary()
      arm_commander.stationary_status_publisher.publish(is_stationary_msg)
      #rospy.loginfo("Published current pose message: %s" % pose_msg)
      rate.sleep()
    rospy.spin()
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    ArmCommander.go_home()
    return

if __name__ == '__main__':
  main()