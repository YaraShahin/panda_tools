# Real-Time Kernel

- Franka Robotic Arm requires strict time constaints for its commands to operate the arm. Therefore, it requires a rt kernel.

## Current version
- To view the current rt kernel version: `uname -a`
- Result: `Linux tkh 5.4.26-rt17 #1 SMP PREEMPT_RT Tue Jan 4 09:39:47 EET 2022 x86_64 x86_64 x86_64 GNU/Linux`
- This is compatible with Ubutu 18.04 and libfranka 6

## Files
- Since the current version is 5.4.26, it's generally safe to move/delete any other version files, such as 4.19 which I currently placed in __backups directory.
- In `./Docs`, you'll find instructions and commands for installing and common troubleshooting for rt kernel against multiple versions 

