#!/bin/bash
# Run localization_logger on each bag in two_cameras_tryouts/tryouts
# Bags are processed in numerical order (tryout_1 -> tryout_30).
# Each run creates logger_csv_N in data/logs/ with the same N.

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_DIR="$PKG_DIR/localization_analysis/data/bags/correct_one_cameras_tryouts/"
WS_INSTALL="$PKG_DIR/../install/setup.bash"

source /opt/ros/humble/setup.bash
source "$WS_INSTALL"

# Kill every running localization_logger instance by command-line match.
# Sends SIGINT first (triggers rclpy's clean shutdown / KeyboardInterrupt),
# waits, then force-kills any survivors with SIGKILL.
kill_all_loggers() {
    pkill -INT -f localization_logger 2>/dev/null || true
    sleep 2
    pkill -KILL -f localization_logger 2>/dev/null || true
    sleep 1
}

mapfile -t BAGS < <(ls -d "$BAGS_DIR"/cc_project_tryout_* | sort -V)
TOTAL=${#BAGS[@]}

echo "Found $TOTAL bags to process"
echo "Logs will be saved to: $PKG_DIR/data/logs/"
echo ""

IDX=0
for bag_dir in "${BAGS[@]}"; do
    IDX=$((IDX + 1))
    bag_name=$(basename "$bag_dir")
    echo "========================================"
    echo "[$IDX/$TOTAL] $bag_name"
    echo "========================================"

    # Ensure no stale loggers from a previous (possibly crashed) run
    kill_all_loggers

    # Start a fresh logger in the background
    ros2 run localization_analysis localization_logger &
    LOGGER_PID=$!

    # Give the node time to initialise and advertise its subscriptions via DDS
    sleep 2

    # Play the bag (blocks until playback is complete)
    ros2 bag play "$bag_dir"

    # Brief pause to let any in-flight messages finish being processed
    sleep 1

    # Shut down the logger: SIGINT first for clean rclpy shutdown,
    # then SIGKILL after a grace period to ensure it actually dies.
    kill -INT "$LOGGER_PID" 2>/dev/null || true
    sleep 2
    kill -KILL "$LOGGER_PID" 2>/dev/null || true
    wait "$LOGGER_PID" 2>/dev/null || true

    echo "Done: $bag_name"
    echo ""
done

echo "========================================"
echo "All $TOTAL bags processed!"
echo "Logs: $PKG_DIR/data/logs/"
echo "Mapping: logger_csv_N corresponds to cc_project_tryout_N"
echo "========================================"
