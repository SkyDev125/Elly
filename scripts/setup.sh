#!/bin/bash
# setup_laptop.sh - Automated "Zero-Friction" Robotics Suite

REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

echo "------------------------------------------------"
echo "Starting Elly OS: Automated Link & Consolidation"
echo "[i] Repo Root: $REPO_DIR"

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
if grep -qiE "(Microsoft|WSL)" /proc/version &> /dev/null; then
    if ! grep -q "LIBGL_ALWAYS_SOFTWARE" ~/.bashrc; then
      echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
      echo "[✔] Applied WSL2 Graphics Fix."
    fi
fi

# 3. BUILD LOCAL ASSETS
# ------------------------------------------------
echo "Step 2: Building local robot description meshes..."
cd "$REPO_DIR"
colcon build --packages-select myagv_description --symlink-install

# 4. CONSOLIDATE ALIASES & ENV VARS
# ------------------------------------------------
echo "Step 3: Updating Master Aliases..."

# Clean up ALL Elly aliases so we don't get duplicates on multiple runs
sed -i '/alias start_rviz/d' ~/.bashrc
sed -i '/alias teleop/d' ~/.bashrc
sed -i '/alias map_/d' ~/.bashrc
sed -i '/alias camera_/d' ~/.bashrc
sed -i '/alias lidar_/d' ~/.bashrc
sed -i '/alias robot_/d' ~/.bashrc
sed -i '/alias elly/d' ~/.bashrc
sed -i '/export NANO_IP=/d' ~/.bashrc
sed -i '/export NANO_USER=/d' ~/.bashrc

# Export variables so our helper scripts know where the robot is
echo "export NANO_IP='$NANO_IP'" >> ~/.bashrc
echo "export NANO_USER='$NANO_USER'" >> ~/.bashrc

# Dashboard & Drive
echo "alias start_rviz='source $REPO_DIR/install/setup.bash && rviz2 -d $REPO_DIR/rviz/elly_dash.rviz'" >> ~/.bashrc
echo "alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'" >> ~/.bashrc

# --- THE REMOTE CONTROL SUITE (The New Brain Toggles) ---
BRAIN="bash ~/scripts/brain.sh"

# Camera Controls
echo "alias camera_on='ssh \$NANO_USER@\$NANO_IP \"$BRAIN camera_on\"'" >> ~/.bashrc
echo "alias camera_off='ssh \$NANO_USER@\$NANO_IP \"$BRAIN camera_off\"'" >> ~/.bashrc

# Lidar & Base Controls
echo "alias lidar_on='ssh \$NANO_USER@\$NANO_IP \"$BRAIN lidar_on\"'" >> ~/.bashrc
echo "alias lidar_off='ssh \$NANO_USER@\$NANO_IP \"$BRAIN lidar_off\"'" >> ~/.bashrc

# 2D Mapping
echo "alias map_2d_on='ssh \$NANO_USER@\$NANO_IP \"$BRAIN map_2d_on\"'" >> ~/.bashrc
echo "alias map_2d_off='ssh \$NANO_USER@\$NANO_IP \"$BRAIN map_2d_off\"'" >> ~/.bashrc

# 3D Mapping (RTAB-Map)
echo "alias map_3d_on='ssh \$NANO_USER@\$NANO_IP \"$BRAIN map_on\"'" >> ~/.bashrc
echo "alias map_3d_off='ssh \$NANO_USER@\$NANO_IP \"$BRAIN map_off\"'" >> ~/.bashrc

# Utilities
echo "alias robot_status='ssh \$NANO_USER@\$NANO_IP \"screen -ls\"'" >> ~/.bashrc
echo "alias robot_peek='ssh -t \$NANO_USER@\$NANO_IP \"screen -r\"'" >> ~/.bashrc

# Map Management
echo "alias map_2d_save='bash $REPO_DIR/scripts/save_room.sh 2d'" >> ~/.bashrc
echo "alias map_2d_load='bash $REPO_DIR/scripts/load_room.sh 2d'" >> ~/.bashrc
echo "alias map_3d_save='bash $REPO_DIR/scripts/save_room.sh 3d'" >> ~/.bashrc
echo "alias map_3d_load='bash $REPO_DIR/scripts/load_room.sh 3d'" >> ~/.bashrc

# --- THE HELP MENU (The elly command) ---
echo "alias elly='echo \"
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

DRIVING & VISUALS:
  teleop         - Keyboard Control (Laptop)
  start_rviz     - Open 3D Dashboard (Laptop)

DIAGNOSTICS & DATA:
  robot_status   - List active Nano processes
  robot_peek [n] - View live logs of a session
------------------------------------------------\"'" >> ~/.bashrc

echo "------------------------------------------------"
echo "Setup Complete! Run: source ~/.bashrc"
echo ""
echo "Type 'elly' at any time to see the command list!"
echo "------------------------------------------------"
