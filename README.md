# panda_tools

Hardware interface package to wrap the standard commands for each robot actuation. Includes standard algorithms related to kinematics such as collision avoidance and some typical examples such as pnp.

## Environment_setup

For more details about environment setup instructions, check docs/dev_env_readme.md and docs/panda_env_readme.md

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

## Package structure

### docs

### config

### launch

### scripts