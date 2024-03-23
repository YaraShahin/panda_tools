#!/usr/bin/env python

import rospy
from panda_tools.msg import GripperAction
from geometry_msgs.msg import Pose

def pnp_fixed():
    # Initialize the ROS node
    rospy.init_node('pnp_demo_fixed', anonymous=True)
    
    # create target pose commander publisher and initialize pose msgs for pick place and place place
    pose_publisher = rospy.Publisher('target_ee_pose', Pose, queue_size=10)
    
    pick_pose = Pose()
    pick_pose.pose.position.x = 0.484
    pick_pose.pose.position.y = 0.333
    pick_pose.pose.position.z = 0.340
    pick_pose.pose.orientation.x = 1
    pick_pose.pose.orientation.y = 0
    pick_pose.pose.orientation.z = 0
    pick_pose.pose.orientation.w = 0
    
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
        pnp_fixed()
    except rospy.ROSInterruptException:
        pass