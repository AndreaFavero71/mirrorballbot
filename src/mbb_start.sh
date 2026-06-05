#!/usr/bin/env bash

#######   Andrea Favero,  05 June 2026  #################################
#
#  This bash script sets the display and starts the MirrorBallBot
#
#########################################################################

#!/bin/bash
LOG="$HOME/mirrorballbot/src/mbb_log.log"

echo "$(date): Starting MirrorBallBot..." >> $LOG

# Find X server
for i in {1..30}; do
    if [ -e /tmp/.X11-unix/X0 ]; then
        break
    fi
    sleep 1
done

export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"

sleep 5

cd "$HOME/mirrorballbot/src"
python3 mbb_gui.py >> $LOG 2>&1
