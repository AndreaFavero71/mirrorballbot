#!/bin/bash

#####################   Andrea Favero,  24 May 2026  ###################################
#
#  MirrorBallBot - System Configuration Script
#
#  First update the system
#     sudo apt update
#     sudo apt upgrade -y
#
#  Then clone the repo  (git clone https://github.com/AndreaFavero71/mirrorballbot.git)
#
#  Finally run this file (bash mbb_install.sh) from the $HOME/mirrorballbot/src folder
#
########################################################################################


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
    echo "  cd $HOME/mirrorballbot/src"
    echo "  bash mbb_install.sh"
    exit 1
fi

# Get the base directory
BASE_DIR="$(cd .. && pwd)"

# Get the username (works for pi, bot, robot, etc.)
USERNAME="$USER"
HOME_DIR="$HOME"

echo "User: $USERNAME"
echo "Home directory: $HOME_DIR"
echo "Repository root: $BASE_DIR"

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
enable_overlay "dtoverlay=vc4-kms-dsi-7inch"
enable_overlay "dtoverlay=gpio-shutdown,gpio_pin=26,gpio_pull=up,active_low=1,debounce=200"
enable_setting "dtparam=i2c_arm" "on"
enable_setting "dtparam=i2c_arm_baudrate" "200000"
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
sudo usermod -a -G i2c,gpio "$USERNAME"


# ============================================================================
# Make startup script executable
# ============================================================================

echo ""
echo "→ Setting executable permission on mbb_start.sh..."
chmod +x "$BASE_DIR/src/mbb_start.sh"


# ============================================================================
# Create desktop shortcut
# ============================================================================

if [ -d "$HOME_DIR/Desktop" ]; then
    echo ""
    echo "→ Creating desktop shortcut..."
    cat > "$HOME_DIR/Desktop/mirrorballbot.desktop" << EOF
[Desktop Entry]
Name=MirrorBallBot
Comment=Launch the Ball Balancing Robot
Exec=lxterminal -e '$BASE_DIR/src/mbb_start.sh'
Icon=$BASE_DIR/src/mbb_icon.ico
Terminal=false
Type=Application
Categories=Utility;Robot;
EOF
    chmod +x "$HOME_DIR/Desktop/mirrorballbot.desktop"
    
    # Mark desktop file as trusted (bypasses confirmation on Bookworm, harmless on Trixie)
    gio set "$HOME_DIR/Desktop/mirrorballbot.desktop" metadata::trusted true 2>/dev/null || true
    
    # Disable confirmation dialog (PCManFM config)
    mkdir -p "$HOME_DIR/.config/pcmanfm/LXDE-pi"
    cat > "$HOME_DIR/.config/pcmanfm/LXDE-pi/pcmanfm.conf" << 'EOF'
[config]
bm_open_method=0

[volume]
show_on_desktop=1

[gui]
confirm_del=1
confirm_trash=1
confirm_execute_file=0
EOF
    
    echo "  Desktop shortcut created"
    echo "  Confirmation dialog disabled"
fi


# ============================================================================
# Configure crontab for auto-start
# ============================================================================

echo ""
echo "→ Configuring crontab for auto-start..."

CRON_LINE="# @reboot bash -l $BASE_DIR/src/mbb_start.sh > $BASE_DIR/src/mbb_log.log 2>&1"

# Add the line only if it doesn't already exist
if ! crontab -l 2>/dev/null | grep -qF "$CRON_LINE"; then
    (crontab -l 2>/dev/null; echo "$CRON_LINE") | crontab -
    echo "  Added crontab entry"
else
    echo "  Crontab entry already exists"
fi


# ============================================================================
# Completion message
# ============================================================================

echo ""
echo "=========================================="
echo "Configuration complete!"
echo ""
echo "User: $USERNAME"
echo "Repository root: $BASE_DIR"
echo ""
echo "What was configured:"
echo "  - Python libraries (8 packages via apt)"
echo "  - I2C enabled (uncommented in config.txt + raspi-config)"
echo "  - DSI display overlay (uncommented in config.txt)"
echo "  - GPU memory set to 128MB"
echo "  - User '$USERNAME' added to i2c and gpio groups"
echo "  - Executable permission set on src/mbb_start.sh"
if [ -f "$HOME_DIR/Desktop/mirrorballbot.desktop" ]; then
    echo "  - Desktop shortcut created"
fi
echo "  - Crontab entry configured (commented by default)"
echo ""
echo "To enable auto-start on boot:"
echo "  crontab -e"
echo "  Remove the '#' at the beginning of the @reboot line"
echo ""
echo "Next steps:"
echo "  1. REBOOT: sudo reboot"
echo "  2. After reboot, test I2C: i2cdetect -y 1"
echo "  3. Test fans: python3 mbb_fans_test.py"
echo "  4. Test motors: python3 mbb_motors_test.py"
echo ""
echo "For other tests connect via a VNC Viewer"
echo "  5. Test camera: python3 mbb_camera.py"
echo "  6. Run robot: python3 mbb_gui.py"
echo "     or double-click the desktop icon"
echo "=========================================="
