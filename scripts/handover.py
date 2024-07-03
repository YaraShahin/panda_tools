#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import time
from panda_tools.msg import GripperAction
from std_msgs.msg import Float32, Int32
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
import math
class Handover:
    def __init__(self):
        rospy.init_node('handover')
        rospy.loginfo("Initiating Handover with panda")
        self.rate = rospy.Rate(1)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_publisher = rospy.Publisher('/target_ee_pose', Pose, latch=True, queue_size=20)
        self.hand_pose_publisher = rospy.Publisher('/hand_pose', PoseStamped, latch=True, queue_size=20)
        self.joint_publisher = rospy.Publisher('/joint_command', Int32, queue_size=10)
        rospy.Subscriber("grasp_width", Float32, self.width_callback)
        self.gripper_publisher = rospy.Publisher('target_gripper_action', GripperAction, queue_size=10)
        self.grip_msg = GripperAction()
        
    def transform_to_pose(self, transform, orientation = 0):
        
        #Convert box orientation to euler form
        #e = self.euler_from_quaternion(transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w)
        #rospy.logerr(e)
        #Convert orientation back to quaternion, keeping only yaw angle
        #q = self.quaternion_from_euler(e[0],e[1],e[2])

        #Converting to quaternion matrix to flip orientation
        #rotation_matrix = quaternion_matrix([q[0],q[1],q[2],q[3]])
        #flipping_matrix = quaternion_matrix(np.array([0, 1, 0, 0]))

        #Flip orientation by dot multiplication
        #new_orientation_matrix = rotation_matrix.dot(flipping_matrix)
        #transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w = quaternion_from_matrix(new_orientation_matrix)
        
        """Convert a transform to a PoseStamped message."""
        pose_msg = Pose()
        pose_msg.position.x = transform.transform.translation.x
        pose_msg.position.y = transform.transform.translation.y + 0.005
        pose_msg.position.z = transform.transform.translation.z + 0.09

        if (orientation):
            pose_msg.orientation = transform.transform.rotation
            pose_msg.orientation.x = transform.transform.rotation.x
            pose_msg.orientation.y = transform.transform.rotation.y
            pose_msg.orientation.z = transform.transform.rotation.z
            pose_msg.orientation.w = transform.transform.rotation.w
        else:
            pose_msg.orientation.x = 1
            pose_msg.orientation.y = 0
            pose_msg.orientation.z = 0
            pose_msg.orientation.w = 0
        
        return pose_msg

    def transform_to_poseStamped(self, transform, orientation = 0):
        """Convert a transform to a PoseStamped message."""
        pose_msg = PoseStamped()
        pose_msg.header = transform.header

        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y + 0.005
        pose_msg.pose.position.z = transform.transform.translation.z + 0.09

        if (orientation):
            pose_msg.pose.orientation = transform.transform.rotation
            pose_msg.pose.orientation.x = transform.transform.rotation.x
            pose_msg.pose.orientation.y = transform.transform.rotation.y
            pose_msg.pose.orientation.z = transform.transform.rotation.z
            pose_msg.pose.orientation.w = transform.transform.rotation.w
        else:
            pose_msg.pose.orientation.x = 1
            pose_msg.pose.orientation.y = 0
            pose_msg.pose.orientation.z = 0
            pose_msg.pose.orientation.w = 0
        
        return pose_msg
    
    def width_callback(self, msg):
        self.width = msg.data
        rospy.loginfo(f"Recieved width command {self.width}")

    def do_a_handover(self):
        rospy.loginfo("Executing One Handover...")
        
        # open
        self.grip_msg.command = True
        self.grip_msg.percentage = 0.05
        self.gripper_publisher.publish(self.grip_msg)
        rospy.loginfo("OPENED")

        # Add hand to scene
        transform = self.tf_buffer.lookup_transform("panda_link0", "hand_centroid", rospy.Time(0), rospy.Duration(20.0))
        hand_pose_msg = self.transform_to_poseStamped(transform)
        self.hand_pose_publisher.publish(hand_pose_msg)

        
        # go to handover location
        transform = self.tf_buffer.lookup_transform("panda_link0", "obj_grasp", rospy.Time(0), rospy.Duration(20.0))
        pose_msg = self.transform_to_pose(transform, 1)
        rospy.loginfo("Found Object Transform of pose:\n" + str(pose_msg))
        self.pose_publisher.publish(pose_msg)
        
        rospy.sleep(5)
        
        # close
        self.grip_msg.command = False
        self.grip_msg.percentage = self.width - 0.015  # the bigger, the more open
        self.gripper_publisher.publish(self.grip_msg)
        rospy.loginfo("CLOSED")
            
        rospy.sleep(5)
        
        # go to home
        rospy.loginfo("Handover Done, Going Back to HOME.")
        self.joint_publisher.publish(1)
        
        rospy.sleep(5)
        
        # go to dropoff
        rospy.loginfo("Going to dropoff.")
        self.joint_publisher.publish(2)
        
        rospy.sleep(5)
        
        # open
        self.grip_msg.command = True
        self.grip_msg.percentage = 0.1           #The bigger, the more open but like different
        self.gripper_publisher.publish(self.grip_msg)
        rospy.loginfo("OPENED")
        
        rospy.sleep(5)
        
        # go to home
        rospy.loginfo("Handover Done, Going Back to HOME.")
        self.joint_publisher.publish(1)
        
        rospy.sleep(5)
        
        rospy.set_param('/handover_status', True)
        

    def run(self):
        while not rospy.is_shutdown():
            handover_status = rospy.get_param('/handover_status', default=True)
            
            if not handover_status:
                self.do_a_handover()
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        handover_commander = Handover()
        handover_commander.run()
    except rospy.ROSInterruptException:
        pass