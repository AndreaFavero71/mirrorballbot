#!/bin/bash

#####################   Andrea Favero,  23 August 2026  ################################
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
# Configure swap space for stability
# ============================================================================

echo ""
echo "→ Configuring swap space..."

# Check current swap size
CURRENT_SWAP=$(swapon --show=Size --bytes | tail -n1 2>/dev/null | numfmt --to=iec)
if [ -z "$CURRENT_SWAP" ] || [ "$CURRENT_SWAP" = "0B" ]; then
    echo "  No swap found - creating 2GB swap file..."
    
    # Create 2GB swap file
    sudo fallocate -l 2G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    
    # Make permanent
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    
    echo ""
    echo "→ Swap space created (2GB)"
else
    echo ""
    echo "→ Swap space already exists: $CURRENT_SWAP"
fi



# ============================================================================
# Install Python packages
# ============================================================================

echo ""
echo "→ Installing Python libraries..."
sudo apt install -y \
    python3-numpy \
    python3-picamera2 \
    python3-gpiozero \
    python3-smbus2 \
    python3-pil \
    python3-pil.imagetk \
    python3-tk



# ============================================================================
# OpenCV Installation with Thorough Cleanup
# ============================================================================

echo ""
echo "→ Installing OpenCV with complete cleanup..."

# Function to check if OpenCV works
check_opencv() {
    python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')" 2>/dev/null
}

# === PHASE 1: AGGRESSIVE CLEANUP ===
# Remove ALL apt OpenCV packages (including dependencies)
echo ""
echo "→ Removing apt packages..."
sudo apt remove -y python3-opencv python3-opencv-apps libopencv* 2>/dev/null || true
sudo apt autoremove -y 2>/dev/null || true
sudo apt autoclean 2>/dev/null || true

# Remove ALL pip OpenCV packages (try multiple package names)
echo ""
echo "→ Removing pip packages..."
pip3 uninstall -y opencv-python opencv-contrib-python opencv-python-headless 2>/dev/null || true
pip3 uninstall -y opencv opencv-contrib opencv-headless 2>/dev/null || true

# Also check pip3 for user installations
pip3 uninstall -y --user opencv-python opencv-contrib-python 2>/dev/null || true

# Find and remove any leftover OpenCV files
echo ""
echo "→ Cleaning up leftover files..."
sudo find /usr/local/lib -name "*opencv*" -type d -exec rm -rf {} + 2>/dev/null || true
sudo find /usr/lib -name "*opencv*" -type d -exec rm -rf {} + 2>/dev/null || true

# Clean pip cache
echo ""
echo "→ Cleaning pip cache..."
pip3 cache purge 2>/dev/null || true

# Clean apt cache completely
echo ""
echo "→ Cleaning apt cache..."
sudo apt clean
sudo apt update

echo ""
echo "→ Cleanup complete"

# === PHASE 2: FRESH INSTALLATION ===
echo ""
# Install from apt (preferred method for Pi)
echo "→ Installing OpenCV from apt repository..."
sudo apt install -y python3-opencv

# === PHASE 3: VERIFICATION ===
if check_opencv > /dev/null; then
    OPENCV_VER=$(check_opencv)
    echo ""
    echo "OpenCV successfully installed: $OPENCV_VER"
    
    # Run a quick test to ensure it's fully functional
    echo ""
    echo "Running functionality test..."
    if python3 -c "import cv2; img = cv2.imread('/dev/null'); print('OpenCV functional')" 2>/dev/null; then
        echo ""
        echo "OpenCV is fully functional"
    else
        echo ""
        echo"OpenCV installed but may have compatibility issues"
    fi
else
    echo ""
    echo "Apt installation failed..."
fi



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
Icon=$BASE_DIR/src/mbb_icon.png
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

# Force crontab initialization (creates the file if it doesn't exist)
crontab -l &>/dev/null || (echo "# MirrorBallBot crontab" | crontab -)

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
echo "  1. Set your preferred VNC (see instructions)"
echo "  2. REBOOT: sudo reboot"
echo "  3. Activate and check your preferred VNC (see instructions)"
echo "  4. Enter the bot folder: cd mirrorballbot/src folder"
echo "  5. Test I2C: i2cdetect -y 1"
echo "  6. Test fans: python3 mbb_fans_test.py"
echo "  7. Test motors: python3 mbb_motors_test.py"
echo ""
echo "For other tests connect via a VNC Viewer"
echo "  8. Test camera: python3 mbb_camera.py"
echo "  9. Run robot: python3 mbb_gui.py"
echo "     or double-click the desktop icon"
echo ""
echo "  ENJOY YOUR NEW ROBOT !"
echo "=========================================="
