#!/usr/bin/env python3

import argparse
import math
import random
from pathlib import Path

# ── Arena ──────────────────────────────────────────────────────────────────────
HALF_X = 30.0          # 
HALF_Y = 5.0

# ── Cylinders ──────────────────────────────────────────────────────────────────
N     = 50
CYL_R = 0.3
CYL_H = 1.0          # same height as the arena walls

# ── Placement rules ────────────────────────────────────────────────────────────
WALL_MARGIN     = CYL_R + 1.0   # keep cylinders away from walls
MIN_CYL_DIST    = CYL_R * 2 + 2.0  # cylinders don't touch each other
SPAWN_X         = 0.0           # robot spawns at the origin
SPAWN_Y         = 0.0
SPAWN_CLEARANCE = 1.5           # no cylinder within this radius of spawn
# ──────────────────────────────────────────────────────────────────────────────


def sample_positions(n: int) -> list[tuple[float, float]]:
    pts   = []
    tries = 0
    while len(pts) < n and tries < 50_000:
        tries += 1
        x = random.uniform(-HALF_X + WALL_MARGIN, HALF_X - WALL_MARGIN)
        y = random.uniform(-HALF_Y + WALL_MARGIN, HALF_Y - WALL_MARGIN)

        if math.hypot(x - SPAWN_X, y - SPAWN_Y) < SPAWN_CLEARANCE:
            continue

        if any(math.hypot(x - px, y - py) < MIN_CYL_DIST for px, py in pts):
            continue

        pts.append((x, y))

    if len(pts) < n:
        raise RuntimeError(
            f"Only placed {len(pts)}/{n} cylinders — "
            "try reducing N, MIN_CYL_DIST, or SPAWN_CLEARANCE."
        )
    return pts


def cylinder_block(i: int, x: float, y: float) -> str:
    return f"""
    <model name="cylinder_{i:02d}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {CYL_H / 2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry>
            <cylinder><radius>{CYL_R}</radius><length>{CYL_H}</length></cylinder>
          </geometry>
        </collision>
        <visual name="v">
          <geometry>
            <cylinder><radius>{CYL_R}</radius><length>{CYL_H}</length></cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out",  required=True, help="Output SDF file path")
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    pts = sample_positions(N)

    print(f"Placed {N} cylinders:")
    for i, (x, y) in enumerate(pts):
        print(f"  cylinder_{i:02d}  ({x:+.2f}, {y:+.2f})")

    cylinders_sdf = "\n".join(cylinder_block(i, x, y) for i, (x, y) in enumerate(pts))

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="arena">

    <!-- Ground -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground%20Plane</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <!-- North wall -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>60 0.1 1</size></box></geometry></collision>
        <visual    name="v"><geometry><box><size>60 0.1 1</size></box></geometry></visual>
      </link>
    </model>

    <!-- South wall -->
    <model name="wall_south">
      <static>true</static>
      <pose>0 -5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>60 0.1 1</size></box></geometry></collision>
        <visual    name="v"><geometry><box><size>60 0.1 1</size></box></geometry></visual>
      </link>
    </model>

    <!-- East wall -->
    <model name="wall_east">
      <static>true</static>
      <pose>30 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.1 10 1</size></box></geometry></collision>
        <visual    name="v"><geometry><box><size>0.1 10 1</size></box></geometry></visual>
      </link>
    </model>

    <!-- West wall -->
    <model name="wall_west">
      <static>true</static>
      <pose>-30 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.1 10 1</size></box></geometry></collision>
        <visual    name="v"><geometry><box><size>0.1 10 1</size></box></geometry></visual>
      </link>
    </model>

    <!-- Randomly placed cylinders -->
    {cylinders_sdf}

  </world>
</sdf>
"""

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(sdf)
    print(f"Wrote: {out.resolve()}")


if __name__ == "__main__":
    main()