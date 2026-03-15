# Quick Start: Running Experiments

## Easiest Way

Use the helper script:

```bash
./run_experiment.sh <num_cylinders> <run_number>
```

## Examples

```bash
# 5 cylinders, run 1
./run_experiment.sh 5 1

# 30 cylinders, run 2  
./run_experiment.sh 30 2

# 45 cylinders, run 3
./run_experiment.sh 45 3
```

## All Available Configurations

| Command | Cylinders | Run |
|---------|-----------|-----|
| `./run_experiment.sh 5 1` | 15 | 1 |
| `./run_experiment.sh 5 2` | 15 | 2 |
| `./run_experiment.sh 5 3` | 15 | 3 |
| `./run_experiment.sh 30 1` | 30 | 1 |
| `./run_experiment.sh 30 2` | 30 | 2 |
| `./run_experiment.sh 30 3` | 30 | 3 |
| `./run_experiment.sh 45 1` | 45 | 1 |
| `./run_experiment.sh 45 2` | 45 | 2 |
| `./run_experiment.sh 45 3` | 45 | 3 |

## Data Output

- Data files are saved to `src/mobile_robot/data/runN.csv` (sequential numbering)
- Each run generates a new file automatically
- You can rename files after each run to track which configuration was used

## Notes

- Coordinate files are in `src/mobile_robot/data/coordinates_*.txt`
- The script automatically finds and uses the correct coordinate file
- Make sure ROS 2 is sourced before running: `source /opt/ros/jazzy/setup.bash`
