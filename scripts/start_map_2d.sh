#!/bin/bash
# start_map_2d.sh - Triggers the Gmapping "Brain" on the Nano

echo "[i] Launching Gmapping over-the-air..."
ssh $NANO_USER@$NANO_IP "bash ~/scripts/brain.sh map_2d_on"
