This is to write the function and interface of all the packages & workspaces involved for each project
1. libfranka (dont include only mention)
2. panda_tools
3. handovers
4. middleware_utils
5. visual_tools
6. realsense_ros (realsense_camera && realsense_camera_description)
7. librealsense (dont include only mention)


# ROS Communication

To setup ROS Noetic Ccommunication between the dev machine and panda machine over a network :

1. Connect between the two machines using a direct network (for example, connect both to phone wifi)
2. Decide which machine will be master. In my case, panda machine is the master
3. For each machine
    1. get ip address: `hostname -I`
    2. In bashrc, add the following lines:
        - `ROS_HOSTNAME=<current-machine-ip>`
        - `ROS_MASTER_URI=http://<panda-machine-ip>:11311`
    3. Open new terminal
4. Type `roscore` in master machine
5. Type `rostopic list` in both machines, they should be the same.

Resources: 

- https://wiki.ros.org/ROS/Tutorials/MultipleMachines
- https://wiki.ros.org/ROS/NetworkSetup

