#!/usr/bin/env bash

#######   Andrea Favero,  23 May 2026  ##################################
#
#  This bash script set the display and starts the MirrorBallBot
#
#########################################################################

#!/bin/bash
LOG="/home/pi/mirrorballbot/src/mbb_log.log"

echo "$(date): Starting MirrorBallBot..." >> $LOG

# Find X server
for i in {1..30}; do
    if [ -e /tmp/.X11-unix/X0 ]; then
        break
    fi
    sleep 1
done

export DISPLAY=:0
export XAUTHORITY=/home/pi/.Xauthority

sleep 5

cd /home/pi/mirrorballbot/src
python3 mbb_gui.py >> $LOG 2>&1