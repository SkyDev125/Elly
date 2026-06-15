#!/bin/bash
# brain.sh - Remote process switchboard for the Jetson Nano.

GALACTIC="source /opt/ros/galactic/setup.bash"
MYAGV="source ~/myagv_ros2/install/setup.bash"
ASTRA="source ~/ros2_astra_ws/install/setup.bash"
MOTION_DIR="$HOME/scripts"
MOTION_SOCKET="/tmp/elly_motion.sock"

session_exists() {
  screen -ls 2>/dev/null | grep -q "[.]$1[[:space:]]"
}

require_no_motion() {
  if motion_active; then
    echo "[x] A movement cue is running. Stop it with elly_move_stop first."
    exit 1
  fi
}

motion_client() {
  python3 "$MOTION_DIR/brain.py" "$@"
}

motion_service_ready() {
  session_exists motion_service && [ -S "$MOTION_SOCKET" ] && motion_client status >/dev/null 2>&1
}

motion_active() {
  if motion_service_ready; then
    # Queries status and checks if it's active/busy
    python3 "$MOTION_DIR/brain.py" status | grep -q "State: Busy"
  else
    false
  fi
}

publish_emergency_stop() {
  timeout 3 bash -lc "$GALACTIC && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'" \
    >/dev/null 2>&1 || true
}

case "${1:-help}" in
  camera_on)
    if session_exists camera; then
      echo "Camera is already running."
      exit 1
    fi
    screen -dmS camera bash -lc "$ASTRA && ros2 launch orbbec_camera astra2_updated.launch.py"
    echo "Camera started."
    ;;

  camera_off)
    screen -XS camera quit 2>/dev/null && echo "Camera stopped." || echo "Camera was not running."
    ;;

  base_on)
    require_no_motion
    if session_exists lidar; then
      echo "[x] lidar_on already owns the robot serial interface. Run lidar_off first."
      exit 1
    fi
    if session_exists base; then
      echo "Base-only driver is already running."
      exit 1
    fi
    screen -dmS base bash -lc "$GALACTIC && $MYAGV && ros2 run myagv_odometry myagv_odometry_node"
    echo "Base-only driver started. LiDAR is not running."
    ;;

  base_off)
    require_no_motion
    screen -XS base quit 2>/dev/null && echo "Base-only driver stopped." || echo "Base-only driver was not running."
    ;;

  lidar_on)
    require_no_motion
    if session_exists base; then
      echo "[x] base_on already owns the robot serial interface. Run base_off first."
      exit 1
    fi
    if session_exists lidar; then
      echo "LiDAR/base driver is already running."
      exit 1
    fi
    screen -dmS lidar bash -lc "$ASTRA && $MYAGV && ros2 launch myagv_odometry myagv_active.launch.py"
    echo "LiDAR/base driver started."
    ;;

  lidar_off)
    require_no_motion
    if session_exists slam_2d; then
      echo "[x] Stop map_2d before stopping the LiDAR/base driver."
      exit 1
    fi
    screen -XS lidar quit 2>/dev/null && echo "LiDAR/base driver stopped." || echo "LiDAR/base driver was not running."
    ;;

  map_on)
    require_no_motion
    CMD="ros2 launch rtabmap_ros rtabmap.launch.py frame_id:=base_footprint subscribe_scan:=true scan_topic:=/scan subscribe_depth:=true approx_sync:=true rgb_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info visual_odometry:=false odom_topic:=/odometry/filtered queue_size:=50 qos_scan:=2 rtabmapviz:=false"
    screen -dmS rtabmap bash -lc "$GALACTIC && $MYAGV && $CMD"
    echo "RTAB-Map (3D) started."
    ;;

  map_off)
    require_no_motion
    screen -XS rtabmap quit 2>/dev/null && echo "RTAB-Map (3D) stopped." || echo "RTAB-Map was not running."
    ;;

  map_2d_on)
    require_no_motion
    if ! session_exists lidar; then
      echo "[x] Start lidar_on before map_2d_on."
      exit 1
    fi
    if session_exists slam_2d; then
      echo "Gmapping is already running."
      exit 1
    fi
    screen -dmS slam_2d bash -lc "$GALACTIC && $MYAGV && ros2 run slam_gmapping slam_gmapping --ros-args -p use_sim_time:=false"
    echo "Gmapping (2D) started with real system time."
    ;;

  map_2d_off)
    require_no_motion
    screen -XS slam_2d quit 2>/dev/null && echo "Gmapping stopped." || echo "Gmapping was not running."
    ;;

  motion_on)
    if motion_service_ready; then
      echo "Movement service is already on."
      exit 0
    fi
    if session_exists motion_service; then
      screen -XS motion_service quit 2>/dev/null || true
    fi
    rm -f "$MOTION_SOCKET"
    if [ ! -f "$MOTION_DIR/brain.py" ]; then
      echo "[x] Persistent movement service is not deployed. Run scripts/setup.sh from the laptop."
      exit 1
    fi
    screen -dmS motion_service bash -lc \
      "$GALACTIC && $MYAGV && python3 '$MOTION_DIR/brain.py' service"
    for _ in $(seq 1 50); do
      if motion_service_ready; then
        echo "Movement service is on and ready."
        exit 0
      fi
      sleep 0.1
    done
    echo "[x] Movement service failed to start."
    exit 1
    ;;

  motion_off)
    if motion_service_ready; then
      motion_client stop_motion || true
      for _ in $(seq 1 50); do
        if ! session_exists motion_service; then
          rm -f "$MOTION_SOCKET"
          publish_emergency_stop
          echo "Movement service is off."
          exit 0
        fi
        sleep 0.1
      done
    fi
    screen -XS motion_service quit 2>/dev/null || true
    rm -f "$MOTION_SOCKET"
    publish_emergency_stop
    echo "Movement service is off."
    ;;

  motion_run)
    cue="${2:-}"
    shift 2 2>/dev/null || true
    if ! motion_service_ready; then
      echo "[x] Movement service is off. Run motion_on first."
      exit 1
    fi
    # Translate old initiate_* cues or call sequence/move directly
    if [ "$cue" = "sequence" ]; then
      motion_client sequence "$@"
    else
      motion_client move "$cue" "$@"
    fi
    ;;

  motion_stop)
    if motion_service_ready; then
      motion_client stop_motion
    else
      publish_emergency_stop
      echo "Movement service is off; zero velocity sent."
    fi
    ;;

  motion_status)
    if motion_service_ready; then
      motion_client status
    else
      echo "Movement service: off"
    fi
    ;;

  nav_off)
    require_no_motion
    screen -XS navigation quit 2>/dev/null && echo "Navigation stopped." || echo "Navigation was not running."
    ;;

  help|*)
    cat <<'EOF'
Elly Jetson commands:
  base_on/base_off
  lidar_on/lidar_off
  camera_on/camera_off
  map_2d_on/map_2d_off
  map_on/map_off
  motion_on/motion_off
  motion_run CUE [args...]
  motion_stop
  motion_status
  nav_off
EOF
    ;;
esac
