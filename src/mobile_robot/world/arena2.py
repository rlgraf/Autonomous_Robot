#!/usr/bin/env python3
"""
arena2.py

Generates a Gazebo SDF world with randomly placed cylinders and
boundary walls with collision geometry (detectable by LiDAR).

This script is intended to be called from a ROS2 launch file:
    python3 arena2.py --out /tmp/arena_generated.sdf

Arena size: 40 m × 10 m
"""

import argparse
import math
import random
from pathlib import Path

# ── Arena bounds (sampling limits only) ───────────────────────────────────────
HALF_X = 20.0   # total length = 40 m
HALF_Y = 5.0    # total width  = 10 m

# ── Cylinders ─────────────────────────────────────────────────────────────────
N = 15
CYL_R = 0.25
CYL_H = 1.00

# ── Placement rules ───────────────────────────────────────────────────────────
WALL_MARGIN = CYL_R + 1.5
MIN_CYL_DIST = CYL_R * 2 + 1.5

# ── Walls with collision geometry ──────────────────────────────────────────────
WALL_HEIGHT = 1.0
WALL_THICKNESS = 0.1

SPAWN_X = 0.0
SPAWN_Y = 0.0
SPAWN_CLEARANCE = 1.5

MAX_TRIES = 50000


def sample_positions(n: int):
    pts = []
    tries = 0

    while len(pts) < n and tries < MAX_TRIES:
        tries += 1

        x = random.uniform(-HALF_X + WALL_MARGIN, HALF_X - WALL_MARGIN)
        y = random.uniform(-HALF_Y + WALL_MARGIN, HALF_Y - WALL_MARGIN)

        # keep robot spawn area clear
        if math.hypot(x - SPAWN_X, y - SPAWN_Y) < SPAWN_CLEARANCE:
            continue

        # keep cylinders separated
        if any(math.hypot(x - px, y - py) < MIN_CYL_DIST for px, py in pts):
            continue

        pts.append((x, y))

    if len(pts) < n:
        raise RuntimeError(
            f"Only placed {len(pts)}/{n} cylinders. "
            "Try reducing N or MIN_CYL_DIST."
        )

    return pts


def cylinder_block(i, x, y):
    return f"""
    <model name="column_{i:02d}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {CYL_H/2:.4f} 0 0 0</pose>

      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{CYL_R}</radius>
              <length>{CYL_H}</length>
            </cylinder>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{CYL_R}</radius>
              <length>{CYL_H}</length>
            </cylinder>
          </geometry>

          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>
    """


def wall_block(name, x, y, z, sx, sy, sz):
    """Create a wall with both visual and collision geometry so LiDAR can detect it."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.0 0.0 1</ambient>
            <diffuse>0.0 0.0 0.0 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>
    """


def build_walls():
    """Build walls with collision geometry so LiDAR can detect them."""
    walls = []

    walls.append(
        wall_block(
            name="wall_north",
            x=0.0,
            y=5.0,
            z=0.5,
            sx=42.0,
            sy=0.1,
            sz=1.0,
        )
    )

    walls.append(
        wall_block(
            name="wall_south",
            x=0.0,
            y=-5.0,
            z=0.5,
            sx=42.0,
            sy=0.1,
            sz=1.0,
        )
    )

    walls.append(
        wall_block(
            name="wall_east",
            x=21.0,
            y=0.0,
            z=0.5,
            sx=0.1,
            sy=10.0,
            sz=1.0,
        )
    )

    walls.append(
        wall_block(
            name="wall_west",
            x=-21.0,
            y=0.0,
            z=0.5,
            sx=0.1,
            sy=10.0,
            sz=1.0,
        )
    )

    return "\n".join(walls)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--out",
        required=True,
        help="Output SDF world file path"
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional random seed for reproducibility"
    )

    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    pts = sample_positions(N)

    print(f"\nPlaced {N} cylinders:")
    for i, (x, y) in enumerate(pts):
        print(f"  column_{i:02d} ({x:+.2f}, {y:+.2f})")

    cylinders_sdf = "\n".join(
        cylinder_block(i, x, y)
        for i, (x, y) in enumerate(pts)
    )

    # walls_sdf = build_walls()

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="arena2">

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground%20Plane</uri>
    </include>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

{cylinders_sdf}

  </world>
</sdf>
"""

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(sdf)

    print("\nGenerated world file:")
    print(out.resolve())


if __name__ == "__main__":
    main()