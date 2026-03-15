#!/bin/bash
# Script to run experiments with different cylinder coordinate configurations
# Usage: ./run_experiment.sh <num_cylinders> <run_number>
# Example: ./run_experiment.sh 5 1

if [ $# -lt 2 ]; then
    echo "Usage: $0 <num_cylinders> <run_number>"
    echo "Example: $0 5 1"
    echo ""
    echo "Available configurations:"
    echo "  - 5 cylinders: runs 1, 2, 3"
    echo "  - 30 cylinders: runs 1, 2, 3"
    echo "  - 45 cylinders: runs 1, 2, 3"
    exit 1
fi

NUM_CYL=$1
RUN_NUM=$2
COORD_FILE="src/mobile_robot/data/coordinates_${NUM_CYL}cyl_run${RUN_NUM}.txt"

if [ ! -f "$COORD_FILE" ]; then
    echo "Error: Coordinate file not found: $COORD_FILE"
    echo ""
    echo "Available coordinate files:"
    ls -1 src/mobile_robot/data/coordinates_*.txt 2>/dev/null || echo "  (none found)"
    exit 1
fi

echo "=========================================="
echo "Running experiment: ${NUM_CYL} cylinders, Run ${RUN_NUM}"
echo "Coordinate file: $COORD_FILE"
echo "=========================================="

# Get absolute path to coordinate file
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
COORD_FILE_ABS="$SCRIPT_DIR/$COORD_FILE"

# Verify file exists with absolute path
if [ ! -f "$COORD_FILE_ABS" ]; then
    echo "Error: Could not resolve absolute path for $COORD_FILE"
    exit 1
fi

echo "Using coordinates from: $COORD_FILE_ABS"
echo ""

# Run the launch file with coordinates
ros2 launch mobile_robot gazebo_model.launch.py coordinates_file:="$COORD_FILE_ABS"
