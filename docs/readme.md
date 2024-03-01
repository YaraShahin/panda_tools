# Environment

## OS

- Image: `lsb_release -a`
    - Distributor ID:	Ubuntu
    <br> Description:	Ubuntu 18.04.6 LTS
    <br> Release:	18.04
    <br> Codename:	bionic
- Kernel: `uname -a`
    - Linux tkh 5.4.26-rt17 #1 SMP PREEMPT_RT Tue Jan 4 09:39:47 EET 2022 x86_64 x86_64 x86_64 GNU/Linux
    - Kernel setups stored in `/home/tkh/kernel`
- Other:
    - ROS: `rosversion -d` 
        - melodic
    - Python:      
        - `python --version`: python2.7
        - `python3 --version`: python3.6
    - pip: `pip --version`
        - pip 21.3.1 from /home/tkh/.local/lib/python3.6/site-packages/pip (python 3.6)

## Main Packages

- libfranka
    - Description: c++ open-source library for direct communication (command and monitor) with panda at 1KHz.
    - Path: `/home/tkh/libfranka`
    - Status: `ldconfig -p | grep libfranka` -> version 8 works
- ros_franka
    - Description: integrates libfranka with ROS & moveit
    - Path: Separate install in each ros ws
- realsense
    - Description:
    - Path:
    - Status:

## Directories

- Structure
    - Kernel: for installation of rt kernel
    - libfranka: for low-level franka library
    - projects: for different project workspaces
    - Docs: for docs and other generic support files and utils
- Instructions
    - Please save your doc files in .md or .txt format (NOT ODT!!)
    - Please keep the `Documents`, `Downloads`, `Music`, and other system directories clean.
    - For any non-project files, please create a `__temp` or a `__backups` directory to let others know that these are not system directories. You need to maintain all directories you created.

