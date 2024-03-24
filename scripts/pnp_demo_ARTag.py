#!/usr/bin/env python

import rospy
from panda_tools.msg import GripperAction
from geometry_msgs.msg import Pose
from tf2_ros import TransformListener
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from src.utils import quaternion_from_euler, euler_from_quaternion
import numpy as np


#Fetches box yaw rotation, flips to point Z-axis down, and returns quaternion array
def flip_z_orientation(rot):
    #Convert box orientation to euler form
    e = euler_from_quaternion(rot[0],rot[1],rot[2],rot[3])

    #Convert orientation back to quaternion, keeping only yaw angle
    q = quaternion_from_euler(0,0,e[2])

    #Converting to quaternion matrix to flip orientation 
    rotation_matrix = quaternion_matrix([q[0],q[1],q[2],q[3]])
    flipping_matrix = quaternion_matrix(np.array([0, 1, 0, 0]))

    #Flip orientation by dot multiplication
    new_orientation_matrix = rotation_matrix.dot(flipping_matrix)
    box_orientation= quaternion_from_matrix(new_orientation_matrix)

    return box_orientation

# TODO See lw ynf3 nfkes ll transform and just take the pose that comes out of the '/ar_pose_marker' marker straight away.
def get_marker_pose():
    '''
    Returns the pose of the marker wrt the panda base link which is panda_link0 or world
    '''
    marker_tf_listener = TransformListener()
    marker_pose = Pose()
    
    #Get Transform from panda base link to AR Tag
    now = rospy.Time.now()
    marker_tf_listener.waitForTransform( "/panda_link0", "/ar_marker_3", now, rospy.Duration(4.0))     #to do: set marker frame name automatically
    (trans, rot) = marker_tf_listener.lookupTransform( "/panda_link0", "/ar_marker_3", now)
    
    rot = flip_z_orientation(rot)
    rospy.loginfo('Transformation (Panda_Link0 => AR Tag):', trans)        

    #Populate Pose Message with appropriate pose from above transform
    marker_pose.pose.position.x = trans[0]
    marker_pose.pose.position.y = trans[1]
    marker_pose.pose.position.z = trans[2] + rospy.get_param('/ArmCommander/hand_EE_offset', 0.11)
    marker_pose.pose.orientation.x = rot[0]
    marker_pose.pose.orientation.y = rot[1]
    marker_pose.pose.orientation.z = rot[2]
    marker_pose.pose.orientation.w = rot[3]
    
    return marker_pose

def pnp_ARTag():
    # Initialize the ROS node
    rospy.init_node('pnp_demo_ARTag', anonymous=True)
    
    # create target pose commander publisher and initialize pose msgs for pick place and place place
    pose_publisher = rospy.Publisher('target_ee_pose', Pose, queue_size=10)
    
    pick_pose = get_marker_pose()
    
    place_pose = Pose()
    home_pose = rospy.get_param("/POSES/HOME")
    place_pose.position.x = home_pose[0]
    place_pose.position.y = home_pose[1]
    place_pose.position.z = home_pose[2]
    place_pose.orientation.x = home_pose[3]
    place_pose.orientation.y = home_pose[4]
    place_pose.orientation.z = home_pose[5]
    place_pose.orientation.w = home_pose[6]
    
    # create gripper commander publisher and initialize gripper command msgs for open and close
    gripper_commander_publisher = rospy.Publisher('target_gripper_action', GripperAction, queue_size=10)
    
    open_grip = GripperAction()
    open_grip.command = True
    open_grip.percentage = 0.08
    
    close_grip = GripperAction()
    close_grip.command = False
    close_grip.percentage = 0.047
    
    # publish the commands
    gripper_commander_publisher.publish(open_grip)
    pose_publisher.publish(pick_pose)
    gripper_commander_publisher.publish(close_grip)
    pose_publisher.publish(place_pose)
    gripper_commander_publisher.publish(open_grip)
    

if __name__ == '__main__':
    try:
        pnp_ARTag()
    except rospy.ROSInterruptException:
        pass