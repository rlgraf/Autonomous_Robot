#!/usr/bin/env python3

import argparse
import math
import random
from pathlib import Path

# ── Arena size ────────────────────────────────────────────────────────────────
HALF_X = 10.0
HALF_Y = 5.0
WALL_THICK = 0.1
WALL_H = 1.0

# ── Characters ────────────────────────────────────────────────────────────────
NUM_ANNAS = 10
NUM_OLAFS = 0

ANNA_MESH_URI = "model://person/anna/anna.gltf"
OLAF_MESH_URI = "model://olaf/olaf.gltf"

# Anna collision (cylinder)
ANNA_R = 0.25
ANNA_H = 1.6

# Olaf collision (box)
OLAF_SX = 0.4
OLAF_SY = 0.4
OLAF_SZ = 1.0

# Placement rules
WALL_MARGIN = max(ANNA_R, OLAF_SX / 2.0) + 0.5   # base margin from walls (collision + safety)
MIN_PERSON_DIST_DEFAULT = 1.5
SPAWN_CLEARANCE_DEFAULT = 1.5

# NEW: keep people away from edges (extra buffer beyond WALL_MARGIN)
EDGE_BUFFER_DEFAULT = 1.5  # meters

MAX_TRIES = 50000


# ──────────────────────────────────────────────────────────────────────────────

def _too_close_to_spawn(x: float, y: float, spawn_clearance: float) -> bool:
    return math.hypot(x, y) < spawn_clearance


def _too_close_to_others(x: float, y: float, pts: list[tuple[float, float]], min_dist: float) -> bool:
    return any(math.hypot(x - px, y - py) < min_dist for px, py in pts)


def sample_positions_interior(n: int, min_dist: float, spawn_clearance: float, edge_buffer: float):
    """
    Place points uniformly in the interior of the arena, staying away from walls.

    Total wall clearance = WALL_MARGIN + edge_buffer.
    """
    pts = []
    tries = 0

    edge_buffer = max(0.0, float(edge_buffer))
    margin = WALL_MARGIN + edge_buffer

    x_min = -HALF_X + margin
    x_max =  HALF_X - margin
    y_min = -HALF_Y + margin
    y_max =  HALF_Y - margin

    if x_min >= x_max or y_min >= y_max:
        raise RuntimeError(
            "edge_buffer too large for this arena size. "
            f"Computed bounds invalid: x[{x_min:.2f},{x_max:.2f}] y[{y_min:.2f},{y_max:.2f}]"
        )

    while len(pts) < n and tries < MAX_TRIES:
        tries += 1

        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)

        # keep robot spawn clear
        if _too_close_to_spawn(x, y, spawn_clearance):
            continue

        # keep people separated
        if _too_close_to_others(x, y, pts, min_dist):
            continue

        pts.append((x, y))

    if len(pts) < n:
        raise RuntimeError(
            f"Not enough space to place characters (placed {len(pts)}/{n}). "
            f"Try reducing --min-dist or --edge-buffer."
        )

    return pts


def anna_block(name: str, x: float, y: float) -> str:
    z_collision = ANNA_H / 2.0   # puts cylinder base at z=0
    visual_sink = -0.8          # lowers only the mesh to avoid hover

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z_collision:.4f} 0 0 0</pose>

      <link name="link">

        <visual name="visual">
          <pose>0 0 {visual_sink:.4f} 0 0 0</pose>
          <geometry>
            <mesh><uri>{ANNA_MESH_URI}</uri></mesh>
          </geometry>
        </visual>

        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{ANNA_R}</radius>
              <length>{ANNA_H}</length>
            </cylinder>
          </geometry>
        </collision>

      </link>
    </model>
    """


def olaf_block(name: str, x: float, y: float) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>

      <link name="link">

        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx> <iyy>0.01</iyy> <izz>0.01</izz>
            <ixy>0.0</ixy> <ixz>0.0</ixz> <iyz>0.0</iyz>
          </inertia>
        </inertial>

        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{OLAF_MESH_URI}</uri>
            </mesh>
          </geometry>
          <pose>0 0 0 0 0 0</pose>
        </visual>

        <collision name="collision">
          <geometry>
            <box>
              <size>{OLAF_SX} {OLAF_SY} {OLAF_SZ}</size>
            </box>
          </geometry>
          <pose>0 0 {OLAF_SZ/2:.4f} 0 0 0</pose>
        </collision>

      </link>
    </model>"""


def main():
    default_out = Path(__file__).parent / "arena2.sdf"

    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=str(default_out))
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--min-dist", type=float, default=MIN_PERSON_DIST_DEFAULT)
    parser.add_argument("--spawn-clearance", type=float, default=SPAWN_CLEARANCE_DEFAULT)
    parser.add_argument(
        "--edge-buffer",
        type=float,
        default=EDGE_BUFFER_DEFAULT,
        help="Extra distance to keep away from walls (meters), beyond WALL_MARGIN."
    )

    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    total = NUM_ANNAS + NUM_OLAFS
    pts = sample_positions_interior(total, args.min_dist, args.spawn_clearance, args.edge_buffer)

    types = ["anna"] * NUM_ANNAS + ["olaf"] * NUM_OLAFS
    random.shuffle(types)

    blocks = []
    anna_i = 0
    olaf_i = 0

    for kind, (x, y) in zip(types, pts):
        if kind == "anna":
            blocks.append(anna_block(f"anna_{anna_i}", x, y))
            anna_i += 1
        else:
            blocks.append(olaf_block(f"olaf_{olaf_i}", x, y))
            olaf_i += 1

    people_sdf = "\n".join(blocks)

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="arena2">

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground%20Plane</uri>
    </include>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- North Wall -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 {HALF_Y} {WALL_H/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{2*HALF_X} {WALL_THICK} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{2*HALF_X} {WALL_THICK} {WALL_H}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- South Wall -->
    <model name="wall_south">
      <static>true</static>
      <pose>0 {-HALF_Y} {WALL_H/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{2*HALF_X} {WALL_THICK} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{2*HALF_X} {WALL_THICK} {WALL_H}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- East Wall -->
    <model name="wall_east">
      <static>true</static>
      <pose>{HALF_X} 0 {WALL_H/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{WALL_THICK} {2*HALF_Y} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{WALL_THICK} {2*HALF_Y} {WALL_H}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- West Wall -->
    <model name="wall_west">
      <static>true</static>
      <pose>{-HALF_X} 0 {WALL_H/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{WALL_THICK} {2*HALF_Y} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{WALL_THICK} {2*HALF_Y} {WALL_H}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>

    {people_sdf}

  </world>
</sdf>
"""

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(sdf)
    print(f"Wrote: {out.resolve()}")


if __name__ == "__main__":
    main()