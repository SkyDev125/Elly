#!/bin/bash
# save_room.sh - Remote save and pull helper

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
MAP_TYPE=${1:-"2d"}
MAP_NAME=${2:-"my_room"}

if [ -z "$NANO_IP" ]; then
    echo "[✘] Error: NANO_IP not set. Run source ~/.bashrc first."
    exit 1
fi

mkdir -p "$REPO_ROOT/maps"

if [ "$MAP_TYPE" == "2d" ]; then
    python3 "$REPO_ROOT/scripts/elly.py" require lidar slam_2d || exit 1
    echo "--- Saving 2D Map Remotely ---"
    # 1. Save on Nano
    ssh $NANO_USER@$NANO_IP "source ~/myagv_ros2/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/$MAP_NAME --ros-args -p map_subscribe_transient_local:=false"
    
    # 2. Pull to Laptop
    echo "--- Syncing 2D Map to Laptop ---"
    scp $NANO_USER@$NANO_IP:~/$MAP_NAME.* "$REPO_ROOT/maps/"

    # 3. Clean up the YAML to use a relative path
    echo "--- Fixing YAML image path ---"
    sed -i "s|image: .*|image: ${MAP_NAME}.pgm|g" "$REPO_ROOT/maps/$MAP_NAME.yaml"

    # 4. Clean up Nano
    ssh $NANO_USER@$NANO_IP "rm ~/$MAP_NAME.*"
    echo "[✔] Success! $MAP_NAME.yaml and $MAP_NAME.pgm saved to $REPO_ROOT/maps/"

elif [ "$MAP_TYPE" == "3d" ]; then
    echo "--- Syncing 3D Database to Laptop ---"
    echo "[!] Ensure you ran 'map_3d_off' first so the database is safely closed!"
    
    # RTAB-Map stores the live map here. We pull it and rename it to your MAP_NAME.
    scp $NANO_USER@$NANO_IP:~/.ros/rtabmap.db "$REPO_ROOT/maps/$MAP_NAME.db"
    echo "[✔] Success! $MAP_NAME.db saved to $REPO_ROOT/maps/"

else
    echo "[✘] Invalid map type. Scripts require '2d' or '3d'."
fi
