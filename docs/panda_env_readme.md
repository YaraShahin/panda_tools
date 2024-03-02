# Environment Setup: Status

## OS

- Image: `lsb_release -a`
    Distributor ID:	Ubuntu
    Description: Ubuntu 20.04.6 LTS
    Release:	20.04
    Codename:	focal

- Kernel: `uname -a`
    - Linux panda-G5-5590 5.9.1-rt20 #1 SMP PREEMPT_RT Thu Feb 8 23:35:51 EET 2024 x86_64 x86_64 x86_64 GNU/Linux
    - Kernel setups stored in `/home/panda/Documents/rt_kerne`
- Other:
    - ROS: `rosversion -d` 
        - noetic
    - Python:      
        - `python3 --version`: python3.8
        - `python2.7 --version` or `python2 --version`: python2.7.18
    - pip: `pip --version`
        - pip 24.0 from /home/panda/.local/lib/python3.8/site-packages/pip (python 3.8)


## Main Packages

- libfranka
    - Description: c++ open-source library for direct communication (command and monitor) with panda at 1KHz.
    - Path: `/home/panda/Documents/libfranka`
    - Version: 0.8.0
- ros_franka
    - Description: integrates libfranka with ROS & moveit
    - Path: Separate install in each ros ws
    - Version: 0.8.0

## Directories

- Structure
    - `~/Documents` for system-wide installations, including the kernel and libfranka
    - `~/panda_ws` for standard franka tasks and examples
    - `~/projects` for different project workspaces
- Instructions
    - Please save your doc files in .md or .txt format (NOT ODT!!)
    - Please keep the `Documents`, `Downloads`, `Music`, and other system directories clean.
    - For any non-project files, please create a `__temp` or a `__backups` directory to let others know that these are not system directories. You need to maintain all directories you created.

# Environment Setup: How-to

## panda machine

### OS

1. Flash Ubuntu 20.04 image:
2. Customize your os via gui
3. Real-time kernel:
    1. See what your current kernel is using `uname -r`
    2. Choose a rt kernel patch closest to yours from: https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/
    Note: It should match your current kernel because older ones or new ones are usually buggy. Your current one should be recent enough. 
    3. Follow the guide: https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel
    4. If you still don't see a grub menu or don't see your new kernel in the grup menu, search for how to update the grub config to add the kernel bin, and also make the timeout > 0.
    5. Use `uname -a` to verify installation
    6. Use `dpkg --list | grep kernel` to see the old kernel modules and `sudo apt purge <module-name>` to remove them.
4. Install ros noetic: https://wiki.ros.org/noetic/Installation/Ubuntu
5. Setup libfranka version 0.8.0 (robot-compatible) by building from source + testing: https://frankaemika.github.io/docs/installation_linux.html#building-from-source
6. Install misc. tools such as:
    - pip: `sudo apt install python3-pip` && `python3 -m pip install pip --upgrade`
    - github ssh key gen

### ROS workspace

1. `mkdir ~/panda_ws && cd ~/panda_ws`
2. `mkdir src && git clone <repo ssh>`
3. Install franka_ros version 0.8.0: 
4. `cd ~/panda_ws && catkin_make`

## ROS Communication

To setup ROS Noetic Ccommunication between the two laptops over a network :
1. 
2. 



# Notes

- Check Compatibility here: https://frankaemika.github.io/docs/compatibility.html

- This pc is only for direct robot control since it requires a realtime kernel. Currently nvidia drivers and cuda are not supported for rt kernel, so we use another pc with Nvidia GPU and cuda for processing, and communicate the extracted image data over ros topics.

- If you want to attempt installing Nvidia & cuda on a rt kernet anyway: https://gist.github.com/pantor/9786c41c03a97bca7a52aa0a72fa9387

- Installing the realtime kernel version that is closest to your non-realtime kernel version is best for development purposes (compatibility with other packages). However, the current kernel version 5.9rt is tested with Ubuntu 20 for panda robot.

# Useful Commands

- Update & Upgrade the System (not recommended for system that have patched compatibility issues):
    - `sudo apt update`
    - `sudo apt upgrade`
    - `sudo apt upgrade --fix-missing --fix-broken`
    - `sudo apt autoremove --purge`

- Package Management
    - see what package you have: `dpkg --list | grep <pkg>`
    - remove it: `sudo apt purge <package-name>`

- misc.
    - `sudo apt install --reinstall linux-headers-$(uname -r)`
    - list hardware: `lspci`
