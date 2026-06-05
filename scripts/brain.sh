#!/bin/bash
# brain.sh - The Switchboard on the Jetson Nano

# Sourcing paths
GALACTIC="source /opt/ros/galactic/setup.bash"
MYAGV="source ~/myagv_ros2/install/setup.bash"
ASTRA="source ~/ros2_astra_ws/install/setup.bash"

case "$1" in
  camera_on)
    screen -dmS camera bash -c "$ASTRA && ros2 launch orbbec_camera astra2_updated.launch.py"
    echo "Camera started."
    ;;
  camera_off)
    screen -XS camera quit && echo "Camera stopped."
    ;;
  lidar_on)
    screen -dmS lidar bash -c "$ASTRA && $MYAGV && ros2 launch myagv_odometry myagv_active.launch.py"
    echo "Lidar/Base started."
    ;;
  lidar_off)
    screen -XS lidar quit && echo "Lidar stopped."
    ;;
  map_on) # This is called by map_3d_on
    CMD="ros2 launch rtabmap_ros rtabmap.launch.py rtabmap_args:='--delete_db_on_start' frame_id:=base_footprint subscribe_scan:=true scan_topic:=/scan subscribe_depth:=true approx_sync:=true rgb_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info visual_odometry:=false odom_topic:=/odometry/filtered queue_size:=50 qos_scan:=2 rtabmapviz:=false"
    screen -dmS rtabmap bash -c "$GALACTIC && $MYAGV && $CMD"
    echo "RTAB-Map (3D) started."
    ;;
  map_off) # This is called by map_3d_off
    screen -XS rtabmap quit && echo "RTAB-Map (3D) stopped."
    ;;
  map_2d_on)
    # Swapped slam_toolbox for the manufacturer's gmapping
    screen -dmS slam_2d bash -c "$GALACTIC && $MYAGV && ros2 launch slam_gmapping slam_gmapping.launch.py"
    echo "Gmapping (2D) started as map_2d."
    ;;
  map_2d_off)
    # Ensure the kill command matches the screen name above
    screen -XS slam_2d quit && echo "Gmapping stopped."
    ;;
esac
