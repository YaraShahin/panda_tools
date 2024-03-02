# Environment Setup: Status

## OS

- Image: `lsb_release -a`
    Distributor ID:	Ubuntu
    Description: Ubuntu 20.04.6 LTS
    Release:	20.04
    Codename:	focal

- Other:
    - ROS: `rosversion -d` 
        - noetic
    - Python:      
        - `python3 --version`: python3.8
        - `python2.7 --version` or `python2 --version`: python2.7.18
    - pip: `pip --version`
        - 


## Main Packages

- librealsense

## Directories

- Structure
    - `~/Documents` for system-wide installations, including librealsense
    - `~/camera_ws` 
    - `` for my current project workspace
- Instructions
    - Please save your doc files in .md or .txt format (NOT ODT!!)
    - Please keep the `Documents`, `Downloads`, `Music`, and other system directories clean.
    - For any non-project files, please create a `__temp` or a `__backups` directory to let others know that these are not system directories. You need to maintain all directories you created.

# Environment Setup: How-to

## panda machine

### OS

1. Flash Ubuntu 20.04 image:
2. Customize your os via gui
3. Update Nvidia Drivers & Install CUDA
4. Install ros noetic: https://wiki.ros.org/noetic/Installation/Ubuntu
5. Install librealsense
6. Install misc. tools such as:
    - pip: `sudo apt install python3-pip` && `python3 -m pip install pip --upgrade`
    - github ssh key gen

### ROS workspace

1. `mkdir ~/project_ws && cd ~/project_ws`
2. `mkdir src && git clone <repo ssh>`
3. `cd ~/project_ws && catkin_make`
