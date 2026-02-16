<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="arena">

    <!-- Basic environment -->
    <gravity>0 0 -9.8</gravity>

    <!-- Lighting -->
    <!-- Add this instead -->
    <light name="sun" type="directional">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 10 0 0 0</pose>
        <diffuse>1 1 1 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
        <attenuation>
            <range>100</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.2 -1</direction>
    </light>

    <!-- Ground -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- ===================== -->
    <!-- Walls (square arena)  -->
    <!-- Arena is centered at (0,0). Change these if you want bigger/smaller. -->
    <!-- ===================== -->
    <!-- Wall parameters -->
    <!-- half-size arena: 5m (so walls at +/-5). wall thickness: 0.2m, height: 1.0m -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 1.0</size></box></geometry>
        </visual>
      </link>
    </model>

    <model name="wall_south">
      <static>true</static>
      <pose>0 -5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 1.0</size></box></geometry>
        </visual>
      </link>
    </model>

    <model name="wall_east">
      <static>true</static>
      <pose>5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 10 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 10 1.0</size></box></geometry>
        </visual>
      </link>
    </model>

    <model name="wall_west">
      <static>true</static>
      <pose>-5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 10 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 10 1.0</size></box></geometry>
        </visual>
      </link>
    </model>

    <!-- ===================== -->
    <!-- Stationary cylinder   -->
    <!-- ===================== -->
    <model name="cylinder_stationary">
      <static>true</static>
      <!-- put it anywhere inside the walls -->
      <pose>-2 1 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.25</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.25</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- ===================== -->
    <!-- Moving cylinder       -->
    <!-- Uses TrajectoryFollower to move between waypoints in XY. -->
    <!-- ===================== -->
    <model name="cylinder_moving">
      <static>false</static>
      <pose>0 0 0.25 0 0 0</pose>

      <link name="base_link">
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.02</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
            <iyy>0.02</iyy> <iyz>0.0</iyz>
            <izz>0.02</izz>
          </inertia>
        </inertial>

        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.25</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.25</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </visual>
      </link>

      <!-- Trajectory follower system plugin -->
      <plugin filename="gz-sim-trajectory-follower-system"
              name="gz::sim::systems::TrajectoryFollower">
        <link_name>base_link</link_name>

        <!-- Make it “push” itself along the path -->
        <force>2.0</force>
        <torque>0.2</torque>

        <!-- How close before a waypoint counts as reached -->
        <range_tolerance>0.15</range_tolerance>

        <!-- Loop around the waypoint list forever -->
        <loop>true</loop>

        <!-- Waypoints are (x y). Keep them inside +/-5 arena (minus cylinder radius). -->
        <waypoints>
          <waypoint>-4 0</waypoint>
          <waypoint> 4 0</waypoint>
        </waypoints>
      </plugin>
    </model>

  </world>
</sdf>
