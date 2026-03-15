# Supervisor Decision Logic - Scenario Analysis

## Current Parameters
- `decision_battery_threshold`: 0.25 (25%)
- `reserve_battery_fraction`: 0.15 (15%)
- `visit_reward`: 1.0
- `extra_distance_weight`: 0.50
- `margin_weight`: 3.0
- `corridor_half_angle_deg`: 35°
- `default_drain_per_meter`: 0.02 (2% per meter)
- `max_object_considered_m`: 10.0m

## Scoring Formula
```
score = visit_reward 
      - (extra_distance_weight * extra_distance)
      + (margin_weight * battery_margin)
```

Where:
- `extra_distance = (distance_to_object + distance_object_to_charger) - distance_to_charger`
- `battery_margin = remaining_usable_battery - energy_needed`
- `energy_needed = total_path_distance * drain_per_meter`
- `remaining_usable = battery_fraction - reserve_battery_fraction`

---

## Scenario 1: Robot at 25% Battery, 2m from Charging Station
**Setup:**
- Battery: 25% (0.25)
- Distance to charger: 2m
- Object: 3m away, 1m further from charger (object-to-charger = 3m)
- Object is within 35° corridor

**Calculations:**
- `remaining_usable = 0.25 - 0.15 = 0.10` (10%)
- `direct_energy = 2m × 0.02 = 0.04` (4%) ✓ Feasible
- `total_path = 3m + 3m = 6m`
- `extra_distance = 6m - 2m = 4m`
- `energy_needed = 6m × 0.02 = 0.12` (12%)
- `margin = 0.10 - 0.12 = -0.02` (NEGATIVE!)

**Result: ❌ REJECTED**
- Reason: Negative margin (not enough battery for round trip)
- Even though close to station, the detour uses too much energy

---

## Scenario 2: Robot at 30% Battery, 3m from Charging Station
**Setup:**
- Battery: 30% (0.30)
- Distance to charger: 3m
- Object: 2m away, 1m further from charger (object-to-charger = 2m)
- Object is within 35° corridor

**Calculations:**
- `remaining_usable = 0.30 - 0.15 = 0.15` (15%)
- `direct_energy = 3m × 0.02 = 0.06` (6%) ✓ Feasible
- `total_path = 2m + 2m = 4m`
- `extra_distance = 4m - 3m = 1m`
- `energy_needed = 4m × 0.02 = 0.08` (8%)
- `margin = 0.15 - 0.08 = 0.07` (7%) ✓ Positive

**Scoring:**
- `score = 1.0 - (0.50 × 1.0) + (3.0 × 0.07)`
- `score = 1.0 - 0.50 + 0.21 = 0.71` ✓ Positive

**Result: ✅ ACCEPTED**
- Good margin, small detour, positive score

---

## Scenario 3: Robot at 25% Battery, 8m from Charging Station
**Setup:**
- Battery: 25% (0.25)
- Distance to charger: 8m
- Object: 5m away, 2m further from charger (object-to-charger = 5m)
- Object is within 35° corridor

**Calculations:**
- `remaining_usable = 0.25 - 0.15 = 0.10` (10%)
- `direct_energy = 8m × 0.02 = 0.16` (16%) ❌ NOT FEASIBLE!

**Result: ❌ REJECTED (Early Exit)**
- Reason: Can't even reach charger directly with remaining battery
- Never evaluates the object visit

---

## Scenario 4: Robot at 40% Battery, 5m from Charging Station
**Setup:**
- Battery: 40% (0.40)
- Distance to charger: 5m
- Object: 4m away, 3m further from charger (object-to-charger = 4m)
- Object is within 35° corridor

**Calculations:**
- `remaining_usable = 0.40 - 0.15 = 0.25` (25%)
- `direct_energy = 5m × 0.02 = 0.10` (10%) ✓ Feasible
- `total_path = 4m + 4m = 8m`
- `extra_distance = 8m - 5m = 3m`
- `energy_needed = 8m × 0.02 = 0.16` (16%)
- `margin = 0.25 - 0.16 = 0.09` (9%) ✓ Positive

**Scoring:**
- `score = 1.0 - (0.50 × 3.0) + (3.0 × 0.09)`
- `score = 1.0 - 1.50 + 0.27 = -0.23` ❌ Negative!

**Result: ❌ REJECTED**
- Reason: Negative score (detour penalty too high)
- Even with positive margin, the 3m detour penalty outweighs the reward

---

## Scenario 5: Robot at 35% Battery, 1m from Charging Station (Near Station Bonus)
**Setup:**
- Battery: 35% (0.35)
- Distance to charger: 1m
- Object: 3m away, 2m further from charger (object-to-charger = 2m)
- Object is within 35° corridor (gets +20° bonus = 55° total)

**Calculations:**
- `remaining_usable = 0.35 - 0.15 = 0.20` (20%)
- `direct_energy = 1m × 0.02 = 0.02` (2%) ✓ Feasible
- `total_path = 3m + 2m = 5m`
- `extra_distance = 5m - 1m = 4m`
- `energy_needed = 5m × 0.02 = 0.10` (10%)
- `margin = 0.20 - 0.10 = 0.10` (10%) ✓ Positive

**Scoring (with near-station bonus):**
- Distance factor: `(10.0 - 1.0) / 10.0 = 0.9`
- `effective_extra_dist_weight = 0.50 × (1.0 - 0.5 × 0.9) = 0.50 × 0.55 = 0.275`
- `score = 1.0 - (0.275 × 4.0) + (3.0 × 0.10)`
- `score = 1.0 - 1.10 + 0.30 = 0.20` ✓ Positive

**Result: ✅ ACCEPTED**
- Near-station bonus reduces detour penalty by 45%
- Makes visit feasible even with larger detour

---

## Scenario 6: Robot at 25% Battery, 15m from Charging Station
**Setup:**
- Battery: 25% (0.25)
- Distance to charger: 15m
- Object: 2m away, 1m further from charger (object-to-charger = 2m)
- Object is within 35° corridor

**Calculations:**
- `remaining_usable = 0.25 - 0.15 = 0.10` (10%)
- `direct_energy = 15m × 0.02 = 0.30` (30%) ❌ NOT FEASIBLE!

**Result: ❌ REJECTED (Early Exit)**
- Reason: Too far from charger, can't reach it directly
- No visits allowed when far from station with low battery

---

## Key Insights

1. **Distance to charger is critical**: If direct path to charger isn't feasible, no visits allowed
2. **Margin must be positive**: Need enough battery for round trip + reserve
3. **Score must be positive**: Even with positive margin, large detours can make score negative
4. **Near-station bonus helps**: Within 10m, reduced penalty makes more visits feasible
5. **Reserve battery is conservative**: 15% reserve means only 10% usable at 25% battery

## Recommendations for Less Strict Logic

1. **Reduce reserve_battery_fraction**: 0.15 → 0.10 (allows more visits)
2. **Increase visit_reward**: 1.0 → 1.5 (makes visits more attractive)
3. **Reduce extra_distance_weight**: 0.50 → 0.30 (less penalty for detours)
4. **Allow small negative margins near station**: e.g., allow -0.02 margin when < 5m from station
