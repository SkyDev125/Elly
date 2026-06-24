#!/bin/bash
# brain.sh - Remote process switchboard for the Jetson Nano.

GALACTIC="source /opt/ros/galactic/setup.bash"
MYAGV="source ~/myagv_ros2/install/setup.bash"
ASTRA="source ~/ros2_astra_ws/install/setup.bash"
MOTION_DIR="$HOME/scripts"
MOTION_SOCKET="/tmp/elly_motion.sock"
MOTION_LOG="$MOTION_DIR/motion_service.log"
CAMERA_COMMAND="ros2 launch orbbec_camera astra2.launch.py uvc_backend:=v4l2 color_width:=640 color_height:=480 color_fps:=10 color_format:=UYVY depth_width:=640 depth_height:=480 depth_fps:=10 enable_ir:=false enable_sync_output_accel_gyro:=false time_domain:=device"
ROS_GRAPH_ENV="$GALACTIC && $MYAGV && $ASTRA"

session_exists() {
  screen -ls 2>/dev/null | grep -q "[.]$1[[:space:]]"
}

service_log() {
  printf '%s/%s.log' "$MOTION_DIR" "$1"
}

wait_for_ros_entries() {
  local session="$1"
  local description="$2"
  local list_type="$3"
  local deadline graph_output ready expected
  shift 3
  deadline=$((SECONDS + 60))

  printf 'Waiting for %s' "$description"
  while ((SECONDS < deadline)); do
    if ! session_exists "$session"; then
      echo
      echo "[x] $description exited during startup."
      tail -n 40 "$(service_log "$session")" 2>/dev/null || true
      return 1
    fi

    graph_output="$(timeout 12 bash -lc "$ROS_GRAPH_ENV && ros2 $list_type list" 2>/dev/null || true)"
    ready=1
    for expected in "$@"; do
      if ! printf '%s\n' "$graph_output" | grep -Fq "$expected"; then
        ready=0
        break
      fi
    done
    if [ "$ready" -eq 1 ]; then
      echo " ready."
      return 0
    fi
    printf '.'
    sleep 0.5
  done

  echo
  echo "[x] $description did not become ready within 60 seconds."
  tail -n 40 "$(service_log "$session")" 2>/dev/null || true
  return 1
}

start_ros_service() {
  local session="$1"
  local description="$2"
  local command="$3"
  local list_type="$4"
  local log_file
  shift 4

  if session_exists "$session"; then
    if wait_for_ros_entries "$session" "$description" "$list_type" "$@"; then
      echo "$description is already on and ready."
      return 0
    fi
    screen -XS "$session" quit >/dev/null 2>&1 || true
    return 1
  fi

  log_file="$(service_log "$session")"
  : >"$log_file"
  screen -dmS "$session" bash -lc "$command >'$log_file' 2>&1"
  if wait_for_ros_entries "$session" "$description" "$list_type" "$@"; then
    echo "$description is on and ready."
    return 0
  fi
  screen -XS "$session" quit >/dev/null 2>&1 || true
  return 1
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
    start_ros_service camera "Depth camera" \
      "$ASTRA && $CAMERA_COMMAND" topic \
      /camera/color/image_raw /camera/depth/image_raw
    ;;

  camera_off)
    if session_exists rtabmap; then
      echo "[x] Stop map_3d before stopping the depth camera."
      exit 1
    fi
    screen -XS camera quit 2>/dev/null && echo "Camera stopped." || echo "Camera was not running."
    ;;

  lidar_on)
    require_no_motion
    start_ros_service lidar "LiDAR/base driver" \
      "$ASTRA && $MYAGV && ros2 launch myagv_odometry myagv_active.launch.py" \
      topic /scan /odom
    ;;

  lidar_off)
    require_no_motion
    if session_exists slam_2d; then
      echo "[x] Stop map_2d before stopping the LiDAR/base driver."
      exit 1
    fi
    if session_exists rtabmap; then
      echo "[x] Stop map_3d before stopping the LiDAR/base driver."
      exit 1
    fi
    if session_exists navigation; then
      echo "[x] Stop navigation with nav_off before stopping the LiDAR/base driver."
      exit 1
    fi
    screen -XS lidar quit 2>/dev/null && echo "LiDAR/base driver stopped." || echo "LiDAR/base driver was not running."
    ;;

  map_on)
    require_no_motion
    if ! session_exists lidar; then
      echo "[x] Start lidar_on before map_3d_on."
      exit 1
    fi
    if ! session_exists camera; then
      echo "[x] Start camera_on before map_3d_on."
      exit 1
    fi
    CMD="ros2 launch rtabmap_ros rtabmap.launch.py frame_id:=base_footprint subscribe_scan:=true scan_topic:=/scan subscribe_depth:=true approx_sync:=true rgb_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info visual_odometry:=false odom_topic:=/odometry/filtered queue_size:=50 qos_scan:=2 rtabmapviz:=false"
    start_ros_service rtabmap "RTAB-Map (3D)" \
      "$GALACTIC && $MYAGV && $CMD" node rtabmap
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
    start_ros_service slam_2d "Gmapping (2D)" \
      "$GALACTIC && $MYAGV && ros2 run slam_gmapping slam_gmapping --ros-args -p use_sim_time:=false" \
      node slam_gmapping
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
    : >"$MOTION_LOG"
    screen -dmS motion_service bash -lc \
      "$GALACTIC && $MYAGV && python3 '$MOTION_DIR/brain.py' service >'$MOTION_LOG' 2>&1"
    for _ in $(seq 1 300); do
      if motion_service_ready; then
        echo "Movement service is on and ready."
        exit 0
      fi
      if ! session_exists motion_service; then
        echo "[x] Movement service exited during startup."
        tail -n 40 "$MOTION_LOG" 2>/dev/null || true
        exit 1
      fi
      sleep 0.1
    done
    echo "[x] Movement service did not become ready within 30 seconds."
    tail -n 40 "$MOTION_LOG" 2>/dev/null || true
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
    screen -XS motion_service quit >/dev/null 2>&1 || true
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

  navigation_wait)
    wait_for_ros_entries navigation "Navigation" action /navigate_to_pose
    ;;

  rtabmap_wait)
    wait_for_ros_entries rtabmap "RTAB-Map (3D)" node rtabmap
    ;;

  help|*)
    cat <<'EOF'
Elly Jetson commands:
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
