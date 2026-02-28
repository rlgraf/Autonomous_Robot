# gen_arena_world.py
import argparse
import random
from pathlib import Path

WORLD_NAME = "arena_25x25"
N = 10

# Arena half-size (50 x 50 walls centered at origin)
HALF = 10.0

# Cylinder properties
CYL_R = 0.3
CYL_H = 2.0

# Wall properties
WALL_THICK = 0.2
WALL_H = 2.0

# Placement margin from walls
MARGIN = CYL_R + 0.3

# Optional: avoid cylinders overlapping each other
MIN_DIST = 2 * CYL_R + 0.2

GROUND_MAT = """
          <material>
            <ambient>0.85 0.85 0.85 1</ambient>
            <diffuse>0.85 0.85 0.85 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
"""

CYL_MAT = """
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
"""



def sample_positions(n):
    pts = []
    tries = 0
    while len(pts) < n and tries < 20000:
        tries += 1
        x = random.uniform(-HALF + MARGIN, HALF - MARGIN)
        y = random.uniform(-HALF + MARGIN, HALF - MARGIN)

        ok = True
        for (px, py) in pts:
            if (x - px) ** 2 + (y - py) ** 2 < MIN_DIST ** 2:
                ok = False
                break
        if ok:
            pts.append((x, y))

    if len(pts) < n:
        raise RuntimeError(
            f"Could only place {len(pts)} cylinders without overlaps. "
            f"Lower MIN_DIST or N."
        )
    return pts


def wall_model(name, x, y, yaw, length):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {WALL_H/2} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{length} {WALL_THICK} {WALL_H}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{length} {WALL_THICK} {WALL_H}</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.15 0.15 1</ambient>
            <diffuse>0.8 0.15 0.15 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>

        </visual>
      </link>
    </model>
    """


#def cylinder_model(i, x, y):
#    return f"""
#    <model name="cylinder_{i:02d}">
#      <pose>{x} {y} {CYL_H/2} 0 0 0</pose>
#      <link name="link">
#        <collision name="collision">
#          <geometry>
#            <cylinder><radius>{CYL_R}</radius><length>{CYL_H}</length></cylinder>
#          </geometry>
#        </collision>
#        <visual name="visual">
#          <geometry>
#            <cylinder><radius>{CYL_R}</radius><length>{CYL_H}</length></cylinder>
#          </geometry>
#          {CYL_MAT}
#        </visual>
#      </link>
#    </model>
#    """

def cylinder_model(i, x, y):
    return f"""
    <model name="cylinder_{i:02d}">
      <static>true</static>
      <pose>{x} {y} {CYL_H/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>{CYL_R}</radius><length>{CYL_H}</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>{CYL_R}</radius><length>{CYL_H}</length></cylinder>
          </geometry>
          {CYL_MAT}
        </visual>
      </link>
    </model>
    """



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="Output world SDF path")
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    pts = sample_positions(N)

    walls = "\n".join([
        wall_model("wall_north", 0, +HALF, 0.0, 2 * HALF),
        wall_model("wall_south", 0, -HALF, 0.0, 2 * HALF),
        wall_model("wall_east",  +HALF, 0, 1.57079632679, 2 * HALF),
        wall_model("wall_west",  -HALF, 0, 1.57079632679, 2 * HALF),
    ])

    cylinders = "\n".join(
        cylinder_model(i, x, y) for i, (x, y) in enumerate(pts)
    )

    sdf = f"""<?xml version="1.0" ?>

<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.8 0.9 1 1</background>
    </scene>


    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <direction>-0.5 0.2 -1</direction>
      <attenuation>
        <range>100</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>


    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>

        </visual>
      </link>
    </model>

    {walls}

    {cylinders}

  </world>
</sdf>
"""
    out.write_text(sdf)
    print(f"Wrote: {out.resolve()}")


if __name__ == "__main__":
    main()
