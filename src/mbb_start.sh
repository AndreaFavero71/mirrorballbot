#!/usr/bin/env bash

#######   Andrea Favero,  22 May 2026  ##################################
#
#  This bash script activates the venv, and starts the ballbalance
#
#########################################################################

# Set display
export DISPLAY=:0
export XAUTHORITY=/home/pi/.Xauthority

# enter the folder with the main scripts
cd /home/pi/mirrorballbot/src

# runs the robot main script
python3 mbb_gui.py
