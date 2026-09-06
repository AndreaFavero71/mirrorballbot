#!/usr/bin/env bash

#######   Andrea Favero,  06 September 2026  ############################
#
#  This bash script sets the display and starts the MirrorBallBot
#
#########################################################################

#!/bin/bash

echo ""
echo "Starting MirrorBallBot..."

LOG="$HOME/mirrorballbot/src/mbb_log.log"
echo "$(date): Starting MirrorBallBot..." >> $LOG

# Find X server
for i in {1..30}; do
    if [ -e /tmp/.X11-unix/X0 ]; then
        echo "X server found (attempt $i)" >> $LOG
        break
    fi
    echo "Waiting for X server (attempt $i/30)..." >> $LOG
    sleep 1
done

# Check if X server was found
if [ ! -e /tmp/.X11-unix/X0 ]; then
    echo "ERROR: X server not found after 30 seconds" >> $LOG
    echo "ERROR: X server not found after 30 seconds"
    exit 1
fi

export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"

cd "$HOME/mirrorballbot/src"

# Log start
echo "$(date): Starting mbb_gui.py" >> $LOG

# Update status on Terminal
echo "MirrorBallBot started."

# Run with output redirection
python3 mbb_gui.py >> $LOG 2>&1

# Log exit
echo "$(date): MirrorBallBot exited with code $?" >> $LOG

# Update status on Terminal
echo "MirrorBallBot stopped."
sleep 1
