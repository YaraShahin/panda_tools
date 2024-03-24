#!/usr/bin/env python

import rospy
from panda_tools.msg import GripperAction
from geometry_msgs.msg import Pose
from std_msgs.msg import Header, Bool

def pnp_fixed():
    global ready
    pose_publisher = rospy.Publisher('target_ee_pose', Pose, queue_size=10)
    rospy.Subscriber("/stationary_status", Bool, status_callback)
    gripper_commander_publisher = rospy.Publisher('target_gripper_action', GripperAction, queue_size=10)
    
    # Initialize the ROS node
    rospy.init_node('pnp_demo_fixed', anonymous=True)
    rate = rospy.Rate(10)
    
    # create target pose commander publisher and initialize pose msgs for pick place and place place
    pick_pose = Pose()
    pick_pose.position.x = 0.45
    pick_pose.position.y = -0.25
    pick_pose.position.z = 0.45
    pick_pose.orientation.x = 1
    pick_pose.orientation.y = 0
    pick_pose.orientation.z = 0
    pick_pose.orientation.w = 0
    
    place_pose = Pose()
    home_pose = rospy.get_param("/EE_POSES/HOME")
    print(home_pose[0])
    
    place_pose.position.x = 0.4887510969876708
    place_pose.position.y = -0.1379379308715513
    place_pose.position.z = 0.5133463855136366
    place_pose.orientation.x = -0.995388846659009
    place_pose.orientation.y = 0.046040142208922046
    place_pose.orientation.z = 0.021568293212935644
    place_pose.orientation.w = 0.08133976874881703 
    
    # create gripper commander publisher and initialize gripper command msgs for open and close
    open_grip = GripperAction()
    open_grip.command = True
    open_grip.percentage = 0.08
    
    close_grip = GripperAction()
    close_grip.command = False
    close_grip.percentage = 0.047
    
    # publish the commands
    gripper_commander_publisher.publish(open_grip)

    while (not ready):
        rospy.sleep(3)
    pose_publisher.publish(pick_pose)
    
    gripper_commander_publisher.publish(close_grip)
    
    while (not ready):
        rospy.sleep(3)
    pose_publisher.publish(place_pose)
    
    gripper_commander_publisher.publish(open_grip)
    
def status_callback(msg):
    global ready
    ready = msg.data

if __name__ == '__main__':
    global ready
    ready = True
    try:
        pnp_fixed()
    except rospy.ROSInterruptException:
        pass