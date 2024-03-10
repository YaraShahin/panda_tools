#!/usr/bin/env python

import rospy
import actionlib
from franka_gripper.msg import MoveAction
from franka_gripper.msg import HomingAction
from franka_gripper.msg import MoveActionGoal     # imported to populate goal with the correct syntax
from tf.transformations import quaternion_matrix, quaternion_from_matrix

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
        self.move_action(offset_maths, 0.02)  # width, speed

    def open(self, diameter):
        offset_maths = (diameter - self.finger_offset + self.grasp_clearence)
        self.move_action(offset_maths, 0.02)  # width, speed


def main():
    grippers = GripperClient()
    print("here1")
    grippers.open(0.2)
    print("here2")
    grippers.close(0.2)

if __name__ == '__main__':
    rospy.init_node('Move_Robot')
    try:
        main()
    except Exception as e:
        pass