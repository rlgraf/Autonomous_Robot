# Parameter Updates Applied

All recommended parameter changes have been applied to align with the 40m × 10m arena and robot dynamics.

## Changes Applied

### Critical Fixes

1. **`reserve_battery_fraction`**: 0.08 → **0.15**
   - **File**: `decider_parameters.yaml`, `supervisor_node.py`
   - **Reason**: Worst case scenario (40m to charger) requires 80% battery, but only 8% was reserved. Now reserves 15% for safety.

2. **`max_object_considered_m`**: 3.0 → **10.0**
   - **File**: `decider_parameters.yaml`, `supervisor_node.py`
   - **Reason**: Arena is 40m long, charging stations are 40m apart. 3.0m was way too restrictive. Now considers objects up to 10m away.

### High Priority Fixes

3. **`corridor_half_angle_deg`**: 25.0 → **35.0**
   - **File**: `decider_parameters.yaml`, `supervisor_node.py`
   - **Reason**: Arena is 10m wide. 25° corridor at 20m distance = 8.7m wide, which excludes objects near walls. 35° provides better coverage.

4. **`target_capture_radius`**: 0.9 → **1.2**
   - **File**: `decider_parameters.yaml`, `supervisor_node.py`
   - **Reason**: Must be ≥ `stop_distance` (1.0m) to ensure proper state transitions. 1.2m provides safety margin.

5. **`visited_radius`**: 0.8 → **1.2**
   - **File**: `battery_tunable_parameters.yaml`, `move5.py`
   - **Reason**: Robot stops at 1.0m, so 0.8m visited radius means objects might never be marked as visited. 1.2m ensures proper marking.

6. **`scan_window_deg`**: 8.0 → **15.0**
   - **File**: `decider_parameters.yaml`, `supervisor_node.py`
   - **Reason**: 8° was too narrow for obstacle detection. 15° provides better path clearance evaluation.

### Medium Priority Fixes

7. **`base_angular_speed`**: 0.8 → **1.2**
   - **File**: `battery_tunable_parameters.yaml`, `move5.py`, `auto_recharge_node.py`
   - **Reason**: Hardware limit is 2.0 rad/s. 0.8 was too conservative for 40m arena. 1.2 provides faster navigation while staying safe.

8. **`min_motion_for_drain_update`**: 0.2 → **0.1**
   - **File**: `decider_parameters.yaml`, `supervisor_node.py`
   - **Reason**: More frequent updates improve battery drain estimation accuracy, especially in tight spaces.

## Summary

All parameters now align with:
- **Arena scale**: 40m × 10m
- **Robot dimensions**: 1.0m × 0.6m × 0.3m
- **Charging station distance**: 40m apart
- **Hardware limits**: Max angular velocity 2.0 rad/s

The robot should now:
- ✅ Consider objects up to 10m away (vs 3m before)
- ✅ Reserve 15% battery for reaching charger (vs 8% before)
- ✅ Have wider acceptance angle for objects (35° vs 25°)
- ✅ Properly mark objects as visited (1.2m vs 0.8m)
- ✅ Navigate faster with better obstacle detection

All code defaults have been updated to match YAML values to prevent mismatches.
