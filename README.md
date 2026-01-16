# DC-build18-26
Carnegie Mellon University Build18 2026.

This contains code for Caleb Song, David Cho, David Wang, James Shin, and 
Joshua Lee for the team DC is the Best.
Contains code for an application to interface with an autonomous bot that can 
detect and change its course to successfully deliver a product from peer to 
peer. 

Code for microcontroller (STM32) is adapted largely from Caleb's project in 
18-349 Introduction to Embedded Systems.

""normally was in python 3.12""
""conda deactivate"" : to deactivate current env
Env for AI_module demo:
conda create -n cv_test python=3.11 -y
conda activate cv_test
pip install opencv-python
python demo_distance.py

MICRO_CODE SECTION
How to flash the stm32:
In one terminal window, cd into Micro_code/

Then run this command: ./osx_ocd (for apple os)

if that doesn't work then update the permission: chmod +x ./osx_ocd

Then in a separate terminal window cd into Micro_code/ 

type <make> and it'll list all the commands, make flash USER_PROJ will load the user code (user_src) onto the stm32 

FOR RPi4
CAMEERA SECTION

source ~/b18env/bin/activate

Works pretty well on one face, need to test multiple and it can detect faces at 
most around 150 cm away which should be good enough and can detect faces 27 meters close.

