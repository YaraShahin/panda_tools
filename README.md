# panda_tools

Hardware interface package to wrap the standard commands for panda robot. Includes standard algorithms related to kinematics such as collision avoidance and some typical demos such as pnp.

## Environment_setup

 For more details about environment setup instructions, check `docs/env_setup`

 ### Hardware environment

 - Franka Emika Cobot connected via Ethernet to panda machine (G5 dell laptop)
 - RealSense D435 Camera connected via USB to dev machine (g3 dell laptop)
 - ROS communication between the two laptops

 ### Software environment

 - panda machine:
    - Ubuntu 20 + 5.9 realtime kernel
    - ros Noetic

 - dev machine:
    - Ubuntu 20
    - ros Noetic
    - CUDA 11

### Demo

 STEP 1: `roslaunch panda_tools launch_panda.launch`
 STEP 1: `rosrun panda_tools pnp_demo_fixed.py`

## Interfaces

 - `ArmCommader`
    - `target_ee_pose`: Send command for the robot end effector to go to a specific `geometrymsgs.Pose` through publishing a message on this topic.
    - `current_ee_pose`: Get the current end effector pose by reading messages on this topic.

 - `GripperCommander`
    - `target_gripper_action`: Send command for the robot gripeer to open/close by a specific percentage through sending a message of type `GripperAction.msg` to this topic.

    ```
    bool command    # True for opening gripper, False for closing it
    float32 percentage   # gripper fingers percentage
    ```

## Package structure

 ### docs

 TODO

 ### config

 TODO

 ### launch

 TODO

 ### scripts

 TODO

 ### src

 TODO

## Tests
 To verify that the full package works as expected follow these steps:

 1. `roslaunch panda_tools launch_panda.launch`
 Observe that there are no errors in the ros log
 Expected behavior: rviz and moveit planner should open up and the current (Real) robot state should be displayed.

 2. **ArmCommander**
    2.A. `rosrun panda_tools arm_demo.py`
    Expected behavior: the pose defined in the script should appear on Rviz, a trajectory should be generated waiting for your approval, and then afterwards, the robot should go.
    2.B. `rostopic echo current_ee_pose`
    Expected behavior: you should accurately see the current pose of the robot's end effector. You can visualize it on Rviz to make sure.

 3. **GripperCommander**
    `rosrun panda_tools gripper_demo.py`
    Expected Behavior: the gripper should open, then close (with prints).

 4. **pick and place demo**
    `rosrun panda_tools pnp_demo_fixed.py`
    Expected Behavior: The robot should open, go to a pose (confirms first), closes, go to a second pose (confirms first), then closes.

 
