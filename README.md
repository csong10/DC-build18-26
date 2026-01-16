# DC-build18-26
Carnegie Mellon University Build18 2026.

This contains code for Caleb Song, David Cho, David Wang, James Shin, and 
Joshua Lee for the team DC is the Best.
Contains code for an application to interface with an autonomous bot that can 
detect and change its course to successfully deliver a product from peer to 
peer. 

Code for microcontroller (STM32) is adapted largely from Caleb's project in 
18-349 Introduction to Embedded Systems.

THINGS CALEB HAS LEARNED IN MAKING THIS WORK:

_TO SSH 2 RPi4_
Terminal: ssh calebson@192.168.0.126
enter password when prompted

If it doesn't work search up how to do it headless but if we have the monitor: hostname -I reveals IP

_TO CAMERA_
Terminal: source ~/b18env/bin/activate 
to activate venv and then cd into Desktop/build18/DC-build18-26/AI_module/
python3 demo_distance.py
press 'q' to exit
#NOTES works well on one face for now from 20 cm - 140 cm

_TO DRIVE MOTORS_
Must be in Micro_code/ to do the rest
2 terminals: (1) ./osx_ocd (make sure you check your permissions)
(2) make flash (all code will be in kernel mode, uart interrupts are directly & immed serviced)
Edit kernel & motor_driver.c and related files to achieve desired result


