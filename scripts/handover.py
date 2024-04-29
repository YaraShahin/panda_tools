#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import time
from panda_tools.msg import GripperAction
from std_msgs.msg import Float32

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
    """pose_msg.orientation.x = transform.transform.rotation.x
    pose_msg.orientation.y = transform.transform.rotation.y
    pose_msg.orientation.z = transform.transform.rotation.z
    pose_msg.orientation.w = transform.transform.rotation.w"""
    
    pose_msg.orientation.x = 1
    pose_msg.orientation.y = 0
    pose_msg.orientation.z = 0
    pose_msg.orientation.w = 0
    
    return pose_msg

def width_callback(msg):
    global width
    width = msg.data
    rospy.loginfo(f"Recieved width command {width}")

def main():
    global width
    rospy.init_node('handover')
    rospy.loginfo("Initiating Handover with panda")
    rospy.set_param('~debugging', False)
    rate = rospy.Rate(1)  # Hz

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pose_publisher = rospy.Publisher('/target_ee_pose', Pose, latch=True, queue_size=20)
    rospy.Subscriber("grasp_width", Float32, width_callback)
    gripper_publisher = rospy.Publisher('target_gripper_action', GripperAction, queue_size=10)
    grip_msg = GripperAction()
    
    # open
    grip_msg.command = True
    grip_msg.percentage = 0.05
    gripper_publisher.publish(grip_msg)
    rospy.loginfo("OPENED")
    
    transform = tf_buffer.lookup_transform("panda_link0", "obj_grasp", rospy.Time(), rospy.Duration(5.0))
    pose_msg = transform_to_pose(transform)
    print("Found Object Transform of pose:\n" + str(pose_msg))
    pose_publisher.publish(pose_msg)
    
    rospy.sleep(10)
    
    # close
    grip_msg.command = False
    grip_msg.percentage = width - 0.015  # the bigger, the more open
    gripper_publisher.publish(grip_msg)
    rospy.loginfo("CLOSED")
        
    rospy.sleep(5)
    
    print("Handover Done, Going Back to HOME.")
    pose_publisher.publish(home_pose_msg)
    
    rospy.sleep(10)
    
    # open
    grip_msg.command = True
    grip_msg.percentage = 0.1           #The bigger, the more open but like different
    gripper_publisher.publish(grip_msg)
    rospy.loginfo("OPENED")

if __name__ == '__main__':
    main()

