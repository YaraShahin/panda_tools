#!/usr/bin/env python

import rospy
import actionlib
from franka_gripper.msg import MoveAction
from franka_gripper.msg import HomingAction
from franka_gripper.msg import MoveActionGoal     # imported to populate goal with the correct syntax
from panda_tools.msg import GripperAction

class GripperCommander():
    def __init__(self):
        rospy.init_node('gripper_commander', anonymous=True)
        
        self.close_percentage = rospy.get_param("/GripperCommander/close_percentage",0.65)
        self.grasp_clearence = rospy.get_param("/GripperCommander/grasp_clearence",0.03)
        self.finger_offset = rospy.get_param("/GripperCommander/finger_offset",0.01)
        self.speed = rospy.get_param("/GripperCommander/speed", 0.02)
        
        # To command the gripper to open or close by a percentage, just publish a msg to this topic
        rospy.Subscriber("/target_gripper_action", GripperAction, self.gripper_commander_callback)
        
        self.homing_action()
        rospy.sleep(2)
        rospy.loginfo("Gripper Commander Ready to recieve open/close messages!")
        
    def gripper_commander_callback(self, msg):
        rospy.loginfo("Received gripper command message: %s" % msg)
        command, percentage = msg.command, msg.percentage
        if (command):
            self.open(percentage)
        else:
            self.close(percentage)

    def homing_action(self):
        client = actionlib.SimpleActionClient("/franka_gripper/homing", HomingAction)
        client.wait_for_server()
        client.send_goal(True)
        homing_done = client.wait_for_result()
        
        return homing_done

    def move_action(self, w, s):
        client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        client.wait_for_server()
        goal = MoveActionGoal
        goal.width = w
        goal.speed = s
        client.send_goal(goal)
        
        move_done = client.wait_for_result()

        return move_done

    def close(self, diameter):
        offset_maths = self.close_percentage * (diameter - self.finger_offset)
        self.move_action(offset_maths, self.speed)  # width, speed

    def open(self, diameter):
        offset_maths = (diameter - self.finger_offset + self.grasp_clearence)
        self.move_action(offset_maths, self.speed)  # width, speed

def main():
  try:
    gripper_commander = GripperCommander()
    rate = rospy.Rate(1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    gripper_commander.homing_action()
    return
    
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"ERROR: {e}")
        pass