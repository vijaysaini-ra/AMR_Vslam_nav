# AMR Simulation Demo (Jetson Orin Nano + RealSense D455 design)

Autonomous mobile robot that **perceives with a depth camera, builds a 3D map on the GPU,
plans with A\*, and drives to goals while avoiding obstacles**, demonstrated in simulation
(Gazebo) because the motor base is not built yet. The same navigation configuration runs
on the real robot.

## Run it

```bash
~/run_sim_demo.sh                 # RViz + autonomous 5-goal tour, prints A* metrics
~/run_sim_demo.sh tour:=false     # interactive: RViz toolbar "2D Goal Pose" -> click a goal
~/run_sim_demo.sh gz_gui:=true    # also show the Gazebo 3D world (heavier)
~/run_sim_demo.sh rviz:=false     # headless; open Foxglove at ws://<jetson-ip>:8765
```

Results of the tour are printed as a table and saved to `~/.ros/amr_demo_results.csv`
inside the container (`/root/.ros` or `/home/admin/.ros`).

Real-hardware perception (no motors needed, carry the camera by hand):
```bash
~/start_robot.sh     # D455 -> cuVSLAM (visual-inertial SLAM) -> nvblox -> Nav2 A*
```

## System architecture

```
            REAL ROBOT                                  SIMULATION
  RealSense D455 (stereo IR + IMU + depth)     Gazebo: simulated D455 RGB-D + IMU
        |                     |                       |                |
   cuVSLAM (GPU)          depth image            wheel odometry    depth image
   map->odom->base_link       |                  odom->base_link       |
        \_____________________|____________________/                   |
                              v                                        v
                 nvblox (GPU): TSDF 3D reconstruction -> 2D ESDF slice
                              v
          Nav2 costmaps (nvblox layer + inflation, robot footprint 0.44 x 0.34 m)
                              v
          Planner: NavFn with use_astar: true   (A* global path)
          Controller: DWB local planner          (velocity commands)
                              v
                          /cmd_vel  -> motors (real) / Gazebo diff-drive (sim)
```

## Packages

| Package | Contents |
|---|---|
| `robot_bringup` | Real robot: `bringup_launch.py` (D455 + cuVSLAM + nvblox + Nav2), `nav2_params.yaml` (A*) |
| `robot_sim` | Simulation: robot URDF (`urdf/amr_sim.urdf.xacro`), world (`worlds/demo_room.sdf`), `sim_demo_launch.py`, RViz layout, `waypoint_demo` tour node |

The simulation **reuses `robot_bringup/config/nav2_params.yaml`**; only the odometry topic
and `use_sim_time` are rewritten at launch.

## Demo world

10 x 8 m room with two partition walls forming an S-shaped route, plus crates and pillars.
The robot starts bottom-left; the tour visits all three rooms and returns home, so A\* has
to route through the openings in the walls.

## What to show in RViz

- **Green line**: A\* global plan. **Orange**: DWB local trajectory.
- **Coloured map**: costmap built live by nvblox from the depth camera (unexplored = grey).
- **3D mesh**: nvblox reconstruction of what the camera has seen.
- **Camera panel**: simulated D455 image.
