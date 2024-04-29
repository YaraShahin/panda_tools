#!/usr/bin/env python

import rospy
from panda_tools.msg import GripperAction

def gripper_commander_publisher():
    gripper_commander_publisher = rospy.Publisher('target_gripper_action', GripperAction, queue_size=10)

    # Initialize the ROS node
    rospy.init_node('gripper_commander_publisher', anonymous=True)
    rate = rospy.Rate(10)
    grip_msg = GripperAction()
    
    # open
    grip_msg.command = True
    grip_msg.percentage = 0.08
    gripper_commander_publisher.publish(grip_msg)
    rospy.loginfo("OPENED")
    
    # close
    grip_msg.command = False
    grip_msg.percentage = 0.047
    gripper_commander_publisher.publish(grip_msg)
    rospy.loginfo("CLOSED")
    
    # open
    grip_msg.command = True
    grip_msg.percentage = 0.08
    gripper_commander_publisher.publish(grip_msg)
    rospy.loginfo("OPENED")
    
    rospy.sleep(5)

if __name__ == '__main__':
    try:
        gripper_commander_publisher()
    except rospy.ROSInterruptException:
        pass
