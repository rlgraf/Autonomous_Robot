# Parameter Mismatches Between Code and YAML Files

## 1. supervisor_node.py vs decider_parameters.yaml

**Mismatches found:**

| Parameter | Code Default | YAML Value | Status |
|-----------|--------------|------------|--------|
| `default_drain_per_meter` | 0.015 | 0.02 | ❌ MISMATCH |
| `drain_alpha` | 0.15 | 0.10 | ❌ MISMATCH |
| `min_motion_for_drain_update` | 0.10 | 0.2 | ❌ MISMATCH |
| `extra_distance_weight` | 0.55 | 0.70 | ❌ MISMATCH |
| `margin_weight` | 2.0 | 2.5 | ❌ MISMATCH |
| `corridor_half_angle_deg` | 35.0 | 25.0 | ❌ MISMATCH |
| `lidar_clearance_margin` | 0.35 | 0.25 | ❌ MISMATCH |
| `scan_window_deg` | 10.0 | 8.0 | ❌ MISMATCH |
| `max_object_considered_m` | 6.0 | 3.0 | ❌ MISMATCH |

**Impact:** The YAML values will override code defaults, but the code defaults suggest different intended behavior.

## 2. move5.py vs battery_tunable_parameters.yaml

**Mismatches found:**

| Parameter | Code Default | YAML Value | Status |
|-----------|--------------|------------|--------|
| `base_angular_speed` | 0.4 | 0.8 | ❌ MISMATCH |

**Impact:** The navigation node will use 0.8 rad/s from YAML, but code suggests 0.4 was intended.

## 3. soft_obstacle_avoidance_node.py vs avoidance_parameters.yaml

**Mismatches found:**

| Parameter | Code Default | YAML Value | Status |
|-----------|--------------|------------|--------|
| `trigger_distance` | 0.70 | 0.55 | ❌ MISMATCH |
| `forward_arc_deg` | 70.0 | 30.0 | ❌ MISMATCH |
| `avoid_angular_spd` | 0.45 | 0.35 | ❌ MISMATCH |
| `avoid_linear_spd` | 0.15 | 0.10 | ❌ MISMATCH |
| `avoid_linear_dist` | 0.40 | 0.45 | ❌ MISMATCH |
| `control_hz` | 20.0 | 25.0 | ❌ MISMATCH |

**Missing in YAML (declared in code):**
- `max_avoid_sec` (default: 8.0)
- `no_progress_sec` (default: 2.5)
- `progress_epsilon_m` (default: 0.05)
- `release_cooldown_sec` (default: 1.0)

**Impact:** Significant differences in avoidance behavior. Missing parameters will use code defaults.

## 4. auto_recharge_node.py vs battery_tunable_parameters.yaml

**Mismatches found:**

| Parameter | Code Default | YAML Value | Status |
|-----------|--------------|------------|--------|
| `arrived_radius` | 0.60 | 0.50 | ❌ MISMATCH |

**Impact:** Robot will consider itself "arrived" at 0.50m instead of 0.60m.

## Recommendations

1. **Update code defaults** to match YAML values (YAML is source of truth)
2. **OR update YAML** to match code defaults (if code defaults are intentional)
3. **Add missing parameters** to YAML files for soft_obstacle_avoidance_node
4. **Document** which is the source of truth for each parameter
