#!/bin/bash
# load_room.sh - Remote deployment of Maps and Configs

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
MAP_TYPE=${1:-"2d"}
MAP_NAME=${2:-"my_room"}

if [ -z "$NANO_IP" ]; then
    echo "[✘] Error: NANO_IP not set. Run source ~/.bashrc first."
    exit 1
fi

if [ "$MAP_TYPE" == "2d" ]; then
    MAP_PATH="$REPO_ROOT/maps/$MAP_NAME.yaml"
    CONFIG_PATH="$REPO_ROOT/config/myagv_nav2.yaml"
    REMOTE_MAP_DIR="/home/er/maps"
    REMOTE_CONFIG_DIR="/home/er/config"

    if [ ! -f "$MAP_PATH" ]; then
        echo "[✘] Error: Map '$MAP_NAME.yaml' not found in $REPO_ROOT/maps/"
        exit 1
    fi

    echo "--- Ensuring Folders exist on Nano ---"
    ssh $NANO_USER@$NANO_IP "mkdir -p $REMOTE_MAP_DIR $REMOTE_CONFIG_DIR"

    echo "--- Deploying 2D Map & Config to Nano ---"
    scp "$REPO_ROOT/maps/$MAP_NAME.yaml" "$REPO_ROOT/maps/$MAP_NAME.pgm" $NANO_USER@$NANO_IP:$REMOTE_MAP_DIR/
    scp "$CONFIG_PATH" $NANO_USER@$NANO_IP:$REMOTE_CONFIG_DIR/

    echo "--- Starting Remote Localization & Navigation (2D) in Background screen 'navigation' ---"
    CMD_LAUNCH="ros2 launch nav2_bringup bringup_launch.py map:=$REMOTE_MAP_DIR/$MAP_NAME.yaml params_file:=$REMOTE_CONFIG_DIR/myagv_nav2.yaml use_sim_time:=false"
    ssh $NANO_USER@$NANO_IP "screen -dmS navigation bash -lc \"source /opt/ros/galactic/setup.bash && source ~/myagv_ros2/install/setup.bash && $CMD_LAUNCH\""
    echo "[✔] Navigation service started in background screen 'navigation'."

elif [ "$MAP_TYPE" == "3d" ]; then
    MAP_PATH="$REPO_ROOT/maps/$MAP_NAME.db"
    
    if [ ! -f "$MAP_PATH" ]; then
        echo "[✘] Error: 3D Database '$MAP_NAME.db' not found in $REPO_ROOT/maps/"
        exit 1
    fi

    echo "--- Deploying 3D Database to Nano ---"
    ssh $NANO_USER@$NANO_IP "mkdir -p ~/.ros"
    # We push it back to the default RTAB-Map location so the node can read it
    scp "$MAP_PATH" $NANO_USER@$NANO_IP:~/.ros/rtabmap.db

    echo "--- Starting 3D Localization in Background screen 'rtabmap' ---"
    # Notice we REMOVED '--delete_db_on_start' and ADDED 'localization:=true'
    CMD_LOC="ros2 launch rtabmap_ros rtabmap.launch.py frame_id:=base_footprint subscribe_scan:=true scan_topic:=/scan subscribe_depth:=true approx_sync:=true rgb_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info visual_odometry:=false odom_topic:=/odometry/filtered queue_size:=50 qos_scan:=2 rtabmapviz:=false localization:=true"
    
    ssh $NANO_USER@$NANO_IP "screen -dmS rtabmap bash -lc \"source /opt/ros/galactic/setup.bash && source ~/myagv_ros2/install/setup.bash && source ~/ros2_astra_ws/install/setup.bash && $CMD_LOC\""
    echo "[✔] 3D Localization started in background screen 'rtabmap'."

else
    echo "[✘] Invalid map type. Scripts require '2d' or '3d'."
fi
