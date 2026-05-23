#!/bin/bash

# MirrorBallBot - System Configuration Script
#
# First update the system
#     sudo apt-get update
#     apt-get upgrade -y
# Then clone the repo
# Finally run this file (bash mbb_install.sh) from from the /mirrorballbot/src folder

set -e


echo ""
echo "=========================================="
echo "MirrorBallBot System Configuration"
echo "=========================================="

# Keep sudo alive to avoid multiple password prompts
sudo -v
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# Check if we're in the correct directory
if [ ! -f "mbb_gui.py" ] || [ ! -f "mbb_install.sh" ]; then
    echo "Error: Please run this script from the src directory"
    echo "  cd /home/pi/mirrorballbot/src"
    echo "  bash mbb_install.sh"
    exit 1
fi

# Get the base directory
BASE_DIR="$(cd .. && pwd)"


# ============================================================================
# Install Python packages
# ============================================================================

echo ""
echo "→ Installing Python libraries..."
sudo apt install -y \
    python3-numpy \
    python3-opencv \
    python3-picamera2 \
    python3-gpiozero \
    python3-smbus2 \
    python3-pil \
    python3-pil.imagetk \
    python3-tk


# ============================================================================
# Configure config.txt (smart uncomment-first approach)
# ============================================================================

CONFIG="/boot/firmware/config.txt"
echo ""
echo "→ Checking config.txt settings..."

# Function: Uncomment if exists as comment, add if missing, update if different
enable_setting() {
    local setting="$1"
    local value="$2"
    local full_line="$setting=$value"
    
    if grep -q "^#$setting=" "$CONFIG"; then
        # Setting exists but commented - uncomment it
        echo "  Uncommenting: $setting"
        sudo sed -i "s/^#$setting=.*/$full_line/" "$CONFIG"
    elif grep -q "^$setting=" "$CONFIG"; then
        # Setting already enabled - check if value needs update
        current_value=$(grep "^$setting=" "$CONFIG" | head -1 | cut -d'=' -f2)
        if [ "$current_value" != "$value" ]; then
            echo "  Updating: $setting from $current_value to $value"
            sudo sed -i "s/^$setting=.*/$full_line/" "$CONFIG"
        else
            echo "  Already correct: $setting"
        fi
    else
        # Setting doesn't exist at all - add it
        echo "  Adding: $full_line"
        echo "$full_line" | sudo tee -a "$CONFIG" > /dev/null
    fi
}

# Function: Uncomment overlay if exists, add if missing
enable_overlay() {
    local overlay="$1"
    
    if grep -q "^#$overlay" "$CONFIG"; then
        # Overlay exists but commented - uncomment it
        echo "  Uncommenting: $overlay"
        sudo sed -i "s/^#$overlay/$overlay/" "$CONFIG"
    elif grep -q "^$overlay" "$CONFIG"; then
        # Overlay already enabled
        echo "  Already enabled: $overlay"
    else
        # Overlay doesn't exist - add it
        echo "  Adding: $overlay"
        echo "$overlay" | sudo tee -a "$CONFIG" > /dev/null
    fi
}

# Apply config.txt settings
enable_setting "dtparam=i2c_arm" "on"
enable_setting "dtparam=i2c_arm_baudrate" "200000"
enable_overlay "dtoverlay=vc4-kms-dsi-7inch"
enable_setting "gpu_mem" "128"


# ============================================================================
# Enable I2C interface
# ============================================================================

echo ""
echo "→ Enabling I2C interface in raspi-config..."
sudo raspi-config nonint do_i2c 0


# ============================================================================
# Set permissions
# ============================================================================

echo ""
echo "→ Setting permissions..."
sudo usermod -a -G i2c,gpio $USER


# ============================================================================
# Make startup script executable
# ============================================================================

echo ""
echo "→ Setting executable permission on mbb_start.sh..."
chmod +x "$BASE_DIR/src/mbb_start.sh"


# ============================================================================
# Create desktop shortcut
# ============================================================================

if [ -d "/home/pi/Desktop" ]; then
    echo "→ Creating desktop shortcut..."
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
    echo "  Desktop shortcut created"
fi

# Disable the "execute file" confirmation dialog in PCManFM
mkdir -p /home/pi/.config/pcmanfm/LXDE-pi
if ! grep -q "confirm_execute_file=0" /home/pi/.config/pcmanfm/LXDE-pi/pcmanfm.conf 2>/dev/null; then
    echo -e "\n[gui]\nconfirm_execute_file=0" >> /home/pi/.config/pcmanfm/LXDE-pi/pcmanfm.conf
    echo "  Desktop shortcut confirmation disabled"
fi


# ============================================================================
# Configure crontab (append only, preserve existing content)
# ============================================================================

echo ""
echo "→ Configuring crontab for auto-start..."

CRON_LINE="@reboot bash -l $BASE_DIR/src/mbb_start.sh > $BASE_DIR/src/mbb_log.log 2>&1"
CRON_LINE_COMMENTED="# $CRON_LINE"

# Check if entry already exists in user's crontab
if crontab -l 2>/dev/null | grep -qF "$CRON_LINE_COMMENTED"; then
    echo "  Crontab entry already exists"
elif crontab -l 2>/dev/null | grep -qF "$CRON_LINE"; then
    echo "  Crontab entry already enabled"
else
    # Append the commented line to existing crontab (preserving all existing content)
    (crontab -l 2>/dev/null; echo "$CRON_LINE_COMMENTED") | crontab -
    echo "  Added commented crontab entry"
fi


# ============================================================================
# Completion message
# ============================================================================

echo ""
echo "=========================================="
echo "Configuration complete!"
echo ""
echo "Repository root: $BASE_DIR"
echo ""
echo "What was configured:"
echo "  - Python libraries (8 packages via apt)"
echo "  - I2C enabled (uncommented in config.txt + raspi-config)"
echo "  - DSI display overlay (uncommented in config.txt)"
echo "  - GPU memory set to 128MB"
echo "  - User added to i2c and gpio groups"
echo "  - Executable permission set on src/mbb_start.sh"
if [ -f "/home/pi/Desktop/mirrorballbot.desktop" ]; then
    echo "  - Desktop shortcut created"
fi
echo "  - Crontab entry configured (commented by default)"
echo ""
echo "To enable auto-start on boot:"
echo "  sudo crontab -e"
echo "  Remove the '#' at the beginning of the @reboot line"
echo "  OR run: sudo crontab -l | sed 's/^# @reboot/@reboot/' | sudo crontab -"
echo ""
echo "Next steps:"
echo "  1. REBOOT: sudo reboot"
echo "  2. After reboot, test I2C: i2cdetect -y 1"
echo "  3. Test camera: python3 mbb_camera.py"
echo "  4. Test fans: python3 mbb_fans_test.py"
echo "  5. Test motors: python3 mbb_motors_test.py"
echo "  6. Run robot: python3 mbb_gui.py"
echo "     or double-click the desktop icon"
echo "=========================================="