# Experiment Configuration Guide

This guide explains how to run experiments with different cylinder coordinate configurations.

## Coordinate Files

All coordinate files are stored in `src/mobile_robot/data/`:
- `coordinates_5cyl_run1.txt` through `coordinates_5cyl_run3.txt` (15 cylinders each)
- `coordinates_30cyl_run1.txt` through `coordinates_30cyl_run3.txt` (30 cylinders each)
- `coordinates_45cyl_run1.txt` through `coordinates_45cyl_run3.txt` (45 cylinders each)

## Running Experiments

### Method 1: Using the helper script (Recommended)

```bash
./run_experiment.sh <num_cylinders> <run_number>
```

Examples:
```bash
# Run 5 cylinders, run 1
./run_experiment.sh 5 1

# Run 30 cylinders, run 2
./run_experiment.sh 30 2

# Run 45 cylinders, run 3
./run_experiment.sh 45 3
```

### Method 2: Using the launch file directly

```bash
ros2 launch mobile_robot gazebo_model.launch.py \
    coordinates_file:="$(pwd)/src/mobile_robot/data/coordinates_5cyl_run1.txt"
```

Or with absolute path:
```bash
ros2 launch mobile_robot gazebo_model.launch.py \
    coordinates_file:="/full/path/to/src/mobile_robot/data/coordinates_5cyl_run1.txt"
```

## Data Output

Each run will generate a data file in `src/mobile_robot/data/`:
- Files are named `run1.csv`, `run2.csv`, etc. (sequential numbering)
- Each file contains experiment data including visited cylinders, battery levels, and timing

## Experiment Matrix

| Cylinders | Runs | Total Configurations |
|-----------|------|---------------------|
| 5         | 3    | 3                    |
| 30        | 3    | 3                    |
| 45        | 3    | 3                    |
| **Total** |      | **9 configurations** |

## Tips

- Make sure to note which coordinate file was used for each data file
- The data logger automatically saves to sequential `runN.csv` files
- You can rename the output files after each run to keep track:
  ```bash
  mv src/mobile_robot/data/run1.csv src/mobile_robot/data/5cyl_run1_data.csv
  ```
