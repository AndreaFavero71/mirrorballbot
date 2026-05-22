#!/bin/bash
# MirrorBallBot - System Configuration Script
# Run AFTER cloning the repository
# Run with: bash mbb_install.sh (from within the src directory)
# Safe to run multiple times (idempotent)

set -e

echo ""
echo "=========================================="
echo "MirrorBallBot System Configuration"
echo "=========================================="
echo ""

# Check if we're in the correct directory (src folder)
if [ ! -f "mbb_gui.py" ] || [ ! -f "mbb_install.sh" ]; then
    echo "Error: Please run this script from the src directory"
    echo "  cd /home/pi/mirrorballbot/src"
    echo "  bash mbb_install.sh"
    exit 1
fi

# Get the base directory (one level up from src)
BASE_DIR="$(cd .. && pwd)"

echo "Repository root: $BASE_DIR"
echo "Source directory: $BASE_DIR/src"

# Update system
echo ""
echo "Updating system... it might take several minutes"
sudo apt update
sudo apt upgrade -y

# Install required Python packages
echo ""
echo "Installing Python libraries..."
sudo apt install -y \
    python3-numpy \
    python3-opencv \
    python3-picamera2 \
    python3-gpiozero \
    python3-smbus2 \
    python3-pil \
    python3-tk

# Configure config.txt (only add missing settings)
CONFIG="/boot/firmware/config.txt"
echo ""
echo "Checking config.txt settings..."

# Function to add setting if missing
add_if_missing() {
    if ! grep -q "^$1" "$CONFIG"; then
        echo "  Adding: $1"
        echo "$1" | sudo tee -a "$CONFIG" > /dev/null
    else
        echo "  Already present: $1"
    fi
}

# I2C settings
add_if_missing "dtparam=i2c_arm=on"
add_if_missing "dtparam=i2c_arm_baudrate=200000"

# DSI display overlay (critical for touchscreen)
add_if_missing "dtoverlay=vc4-kms-dsi-7inch"

# GPU memory
add_if_missing "gpu_mem=128"

# Enable I2C interface in raspi-config
echo ""
echo "Enabling I2C interface in raspi-config..."
sudo raspi-config nonint do_i2c 0

# Add user to I2C and GPIO groups
echo ""
echo "Setting permissions..."
sudo usermod -a -G i2c,gpio $USER

# Ensure mbb_start.sh is executable
echo ""
echo "Setting executable permissions on startup script..."
chmod +x "$BASE_DIR/src/mbb_start.sh"

# Create desktop shortcut
if [ -d "/home/pi/Desktop" ]; then
    echo "Creating desktop shortcut..."
    cat > /home/pi/Desktop/mirrorballbot.desktop << EOF
[Desktop Entry]
Name=MirrorBallBot
Comment=Launch the Ball Balancing Robot
Exec=$BASE_DIR/src/mbb_start.sh
Icon=$BASE_DIR/src/mbb_icon.ico
Terminal=false
Type=Application
Categories=Utility;Robot;
EOF
    chmod +x /home/pi/Desktop/mirrorballbot.desktop
    chown pi:pi /home/pi/Desktop/mirrorballbot.desktop
    echo "Desktop shortcut created"
fi

# Add commented crontab entry for auto-start (if not already present)
echo ""
echo "Adding commented crontab entry for auto-start..."

CRON_LINE="# @reboot bash -l $BASE_DIR/src/mbb_start.sh > $BASE_DIR/src/mbb_log.log 2>&1"

# Check if line already exists in crontab
if sudo crontab -l 2>/dev/null | grep -qF "$CRON_LINE"; then
    echo "Crontab entry already exists"
else
    # Add the commented line to crontab
    (sudo crontab -l 2>/dev/null; echo "$CRON_LINE") | sudo crontab -
    echo "Added commented crontab entry"
fi

echo ""
echo "=========================================="
echo "Configuration complete!"
echo ""
echo "Repository root: $BASE_DIR"
echo ""
echo "What was configured:"
echo "  - Python libraries (7 packages via apt)"
echo "  - I2C enabled (dtparam + raspi-config)"
echo "  - DSI display overlay (vc4-kms-dsi-7inch)"
echo "  - GPU memory set to 128MB"
echo "  - User added to i2c and gpio groups"
echo "  - Executable permission set on src/mbb_start.sh"
if [ -f "/home/pi/Desktop/mirrorballbot.desktop" ]; then
    echo "  - Desktop shortcut created"
fi
echo "  - Commented crontab entry added"
echo ""
echo "To enable auto-start on boot:"
echo "  sudo crontab -e"
echo "  Find the line: # @reboot bash -l $BASE_DIR/src/mbb_start.sh..."
echo "  Remove the '#' at the beginning to uncomment"
echo ""
echo "Next steps:"
echo "  1. REBOOT: sudo reboot"
echo "  2. After reboot, test I2C: i2cdetect -y 1"
echo "  3. Test camera: python3 mbb_camera.py"
echo "  4. Test fans: python3 mbb_fans_test.py"
echo "  5. Test motors: python3 mbb_motors_test.py"
echo "  6. Test robot: python3 mbb_robot.py"
echo "  7. Run robot with its GUI: python3 mbb_gui.py"
echo "     or double-click the desktop icon"
echo "=========================================="