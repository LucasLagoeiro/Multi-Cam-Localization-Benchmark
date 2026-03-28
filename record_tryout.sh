#!/usr/bin/env bash
# Records /info /plan /localization_pose into a numbered rosbag.
# Usage: ./record_tryout.sh [output_dir]
#   output_dir  — where bags are saved (default: ~/rosbags)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${1:-$SCRIPT_DIR/tryouts}"
mkdir -p "$OUTPUT_DIR"

# Determine next tryout number
NUM=1
while [[ -e "$OUTPUT_DIR/cc_project_tryout_$NUM" ]]; do
    ((NUM++))
done
BAG_PATH="$OUTPUT_DIR/cc_project_tryout_$NUM"

echo "Tryout #$NUM will be saved to: $BAG_PATH"
echo ""
read -rsp "Press any key to START recording..." -n1
echo ""
echo "Recording... Press any key to STOP."

ros2 bag record -o "$BAG_PATH" /info /plan /localization_pose &
BAG_PID=$!

read -rsp "" -n1
kill "$BAG_PID"
wait "$BAG_PID" 2>/dev/null

echo ""
echo "Saved: $BAG_PATH"
