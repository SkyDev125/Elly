#!/bin/bash
# setup_laptop.sh - Automated "Zero-Friction" Robotics Suite

REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

echo "------------------------------------------------"
echo "Starting Elly OS: Automated Link & Consolidation"
echo "[i] Repo Root: $REPO_DIR"

# ========================================================
# STEP 0: PRE-FLIGHT SYSTEM AUDIT & ROS 2 GALACTIC INSTALLER
# ========================================================
echo "Step -1: Auditing Host System Architecture..."

# 1. Verify Ubuntu Version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "20.04" ]; then
        echo "[✘] Error: This script requires Ubuntu 20.04 (Focal Fossa). Found version $VERSION_ID."
        exit 1
    fi
    echo "[✔] OS Check Passed: Ubuntu 20.04 detected."
else
    echo "[✘] Error: Cannot determine OS version. /etc/os-release missing."
    exit 1
fi

# 2. Check if ROS 2 Galactic is already installed
if [ -f /opt/ros/galactic/setup.bash ]; then
    echo "[✔] ROS 2 Galactic installation detected at /opt/ros/galactic."
else
    echo "[!] ROS 2 Galactic not found! Starting automated background installation..."
    echo "[i] Requesting sudo privileges for package management."
    
    # Set up ROS 2 apt repository
    sudo apt update && sudo apt install curl gnupg2 lsb-release -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.p/ros2.list > /dev/null
    
    # Install ROS 2 Galactic Desktop and build tools
    sudo apt update
    sudo apt install ros-galactic-desktop python3-colcon-common-extensions python3-rosdep -y
    
    # Initialize rosdep if not already done
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    if [ -f /opt/ros/galactic/setup.bash ]; then
        echo "[✔] ROS 2 Galactic successfully installed!"
    else
        echo "[✘] Installation failed. Please check your network connection or repository logs."
        exit 1
    fi
fi

# 1. SSH AUTO-LINK (Step 0: The "Annoyance Killer")
# ------------------------------------------------
if [ ! -f ~/.ssh/id_rsa ]; then
    echo "[i] No SSH key found. Generating one for you..."
    ssh-keygen -t rsa -N "" -f ~/.ssh/id_rsa
fi

echo "Enter the MyAGV's IP address (e.g., 192.168.0.200):"
read -p ">> " NANO_IP
NANO_USER="er" # Default user for MyAGV

echo "[i] Establishing permanent link with $NANO_USER@$NANO_IP..."
echo "[!] You may have to type the robot password ONE LAST TIME."
ssh-copy-id -o ConnectTimeout=5 $NANO_USER@$NANO_IP

if [ $? -eq 0 ]; then
    echo "[✔] SSH Link Established! No more passwords needed."
else
    echo "[✘] Connection failed. Check the IP and Wi-Fi."
    exit 1
fi

# 1.5 OVER-THE-AIR (OTA) BRAIN DEPLOYMENT
# ------------------------------------------------
echo "Step 1.5: Pushing Elly OS Brain to Jetson Nano..."

# Check and install 'screen' on the Nano if it's missing
echo "[i] Checking Nano dependencies..."
ssh -t $NANO_USER@$NANO_IP "dpkg -l | grep -qw screen || (echo '[!] Screen missing. Installing now...' && sudo apt update && sudo apt install screen -y)"

# Create remote folder, push the local brain.sh, and make it executable
echo "[i] Pushing local brain.sh to Nano..."
ssh $NANO_USER@$NANO_IP "mkdir -p ~/scripts"
scp "$REPO_DIR/scripts/brain.sh" $NANO_USER@$NANO_IP:~/scripts/brain.sh
ssh $NANO_USER@$NANO_IP "chmod +x ~/scripts/brain.sh"
echo "[✔] Brain deployment complete!"

# 2. WSL GRAPHICS FIX
# ------------------------------------------------
echo "Step 2: Applying WSL Fix if necessary"
if grep -qiE "(Microsoft|WSL)" /proc/version &> /dev/null; then
    if ! grep -q "LIBGL_ALWAYS_SOFTWARE" ~/.bashrc; then
      echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
      echo "[✔] Applied WSL2 Graphics Fix."
    fi
fi

# 3. BUILD LOCAL ASSETS
# ------------------------------------------------
echo "Step 3: Building local robot description meshes..."
cd "$REPO_DIR"
source /opt/ros/galactic/setup.bash
colcon build --packages-select myagv_description --symlink-install

# 4. CONSOLIDATE ALIASES & ENV VARS
# ------------------------------------------------
echo "Step 3: Updating Master Aliases..."

# Safely wipe the old Elly block (if it exists) so we don't duplicate
sed -i '/# === ELLY OS START ===/,/# === ELLY OS END ===/d' ~/.bashrc

# The remote command prefix
BRAIN="bash ~/scripts/brain.sh"

# Append the new block cleanly using a Heredoc
cat << EOF >> ~/.bashrc
# === ELLY OS START ===
# Core Environment Sources
source /opt/ros/galactic/setup.bash

export NANO_IP='$NANO_IP'
export NANO_USER='$NANO_USER'

alias start_rviz='source $REPO_DIR/install/setup.bash && rviz2 -d $REPO_DIR/rviz/elly_dash.rviz'
alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'

alias camera_on='ssh \$NANO_USER@\$NANO_IP "$BRAIN camera_on"'
alias camera_off='ssh \$NANO_USER@\$NANO_IP "$BRAIN camera_off"'
alias lidar_on='ssh \$NANO_USER@\$NANO_IP "$BRAIN lidar_on"'
alias lidar_off='ssh \$NANO_USER@\$NANO_IP "$BRAIN lidar_off"'
alias map_2d_on='bash $REPO_DIR/scripts/start_map_2d.sh'
alias map_2d_off='ssh \$NANO_USER@\$NANO_IP "$BRAIN map_2d_off"'
alias map_3d_on='ssh \$NANO_USER@\$NANO_IP "$BRAIN map_on"'
alias map_3d_off='ssh \$NANO_USER@\$NANO_IP "$BRAIN map_off"'

alias robot_status='ssh \$NANO_USER@\$NANO_IP "screen -ls"'
alias robot_peek='ssh -t \$NANO_USER@\$NANO_IP "screen -r"'
alias robot_find_me='ssh \$NANO_USER@\$NANO_IP "source /opt/ros/galactic/setup.bash && ros2 service call /reinitialize_global_localization std_srvs/srv/Empty"'

alias map_2d_save='bash $REPO_DIR/scripts/save_room.sh 2d'
alias map_2d_load='bash $REPO_DIR/scripts/load_room.sh 2d'
alias map_3d_save='bash $REPO_DIR/scripts/save_room.sh 3d'
alias map_3d_load='bash $REPO_DIR/scripts/load_room.sh 3d'

alias elly='echo "
Elly OS - Command Reference
------------------------------------------------
TOGGLES (Nano Background):
  lidar_on/off   - Base Driver & LiDAR
  camera_on/off  - Orbbec Astra Depth Camera
  map_2d_on/off  - SLAM Toolbox (2D)
  map_3d_on/off  - RTAB-Map (3D)

MAP MANAGEMENT:
  map_2d_save / map_2d_load - Manage .yaml/.pgm files
  map_3d_save / map_3d_load - Manage .db database files
  robot_find_me - Trigger Global Localization

DRIVING & VISUALS:
  teleop         - Keyboard Control (Laptop)
  start_rviz     - Open 3D Dashboard (Laptop)

DIAGNOSTICS & DATA:
  robot_status   - List active Nano processes
  robot_peek [n] - View live logs of a session
------------------------------------------------"'
# === ELLY OS END ===
EOF

echo "------------------------------------------------"
echo "Setup Complete! Run: source ~/.bashrc"
echo ""
echo "Type 'elly' at any time to see the command list!"
echo "------------------------------------------------"
