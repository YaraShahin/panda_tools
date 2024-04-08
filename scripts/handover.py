#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import time

home_pose_msg = Pose()
home_pose_msg.position.x = 0.38752098452455286
home_pose_msg.position.y = 0.025867120028349323
home_pose_msg.position.z = 0.6605113319522242
home_pose_msg.orientation.x = 1
home_pose_msg.orientation.y = 0
home_pose_msg.orientation.z = 0
home_pose_msg.orientation.w = 0

def transform_to_pose(transform):
    """Convert a transform to a PoseStamped message."""
    pose_msg = Pose()
    pose_msg.position.x = transform.transform.translation.x
    pose_msg.position.y = transform.transform.translation.y
    pose_msg.position.z = transform.transform.translation.z + 0.1

    #pose_msg.orientation = transform.transform.rotation
    """pose_msg.orientation.x = transform.transform.orientation.x
    pose_msg.orientation.y = transform.transform.orientation.y
    pose_msg.orientation.z = transform.transform.orientation.z
    pose_msg.orientation.w = transform.transform.orientation.w"""
    
    pose_msg.orientation.x = 1
    pose_msg.orientation.y = 0
    pose_msg.orientation.z = 0
    pose_msg.orientation.w = 0
    
    return pose_msg

def main():
    rospy.init_node('handover')
    rospy.loginfo("Initiating Handover with panda")
    rate = rospy.Rate(1)  # Hz

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pose_publisher = rospy.Publisher('/target_ee_pose', Pose, latch=True, queue_size=20)
    
    transform = tf_buffer.lookup_transform("panda_link0", "obj_grasp", rospy.Time(), rospy.Duration(5.0))
    pose_msg = transform_to_pose(transform)
    print("Found Object Transform of pose:\n" + str(pose_msg))
    pose_publisher.publish(pose_msg)
    
    rospy.sleep(5)
    
    print("Handover Done, Going Back to HOME.")
    pose_publisher.publish(home_pose_msg)

if __name__ == '__main__':
    main()

