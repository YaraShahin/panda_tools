#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def target_pose_publisher():
    # Initialize the ROS node
    rospy.init_node('target_pose_publisher', anonymous=True)
    
    # Create a publisher for the Pose message
    pose_publisher = rospy.Publisher('target_ee_pose', Pose, queue_size=10)
    
    # Create a Pose message object
    pose_msg = Pose()
    
    # Set the pose values (assuming a simple scenario)
    pose_msg.position.x = 1.0
    pose_msg.position.y = 2.0
    pose_msg.position.z = 0.0
    pose_msg.orientation.x = 0.0
    pose_msg.orientation.y = 0.0
    pose_msg.orientation.z = 0.0
    pose_msg.orientation.w = 1.0
    
    pose_publisher.publish(pose_msg)

if __name__ == '__main__':
    try:
        target_pose_publisher()
    except rospy.ROSInterruptException:
        pass
