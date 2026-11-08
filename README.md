# RSDIS Final Project - Group 22

Multi-robot simulation with ROS 2 and Gazebo for RSDIS (ISEP, 2026/27): one UAV and two
ground robots in a 36 × 28 m industrial yard with four gas leaks and two radiation sources.
One leak has no matching gas warning sign nearby, and one radiation-labelled barrel has no source.

Uses ROS 2 Jazzy, Gazebo Harmonic and Nav2. The ground robots are a small version of the
Gazebo tutorial robot and the UAV is the X3 quadcopter.
The plan is in `docs/plan.md`.

## Build

```
sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```
ros2 launch g22_hazard_bringup sim.launch.py
```

Starts Gazebo with the UAV (`uav1`) and the two ground robots (`ugv1`, `ugv2`), each with
its own Nav2. Add `headless:=True` to run without the Gazebo window.

In a second terminal:

```
ros2 launch g22_hazard_mission mission.launch.py
```

Starts the simulated gas/radiation sensor on each robot, the UAV mission (survey,
local search, return and landing), the coordinator that sends the closest ground robot
to confirm each hazard, the camera sign detector, the metrics logger and RViz with the hazard map. Add
`show_truth:=true` to also draw the real sources.

Or everything from one launch file (the mission starts 25 s after the simulation):

```
ros2 launch g22_hazard_mission all.launch.py headless:=True show_truth:=true
```

Each run writes a CSV in `~/g22_runs/` with the estimates, the confirmations, their
time and their distance to the real sources.

## Final deliverables

- [English IEEE report](report/main.pdf), with its [LaTeX source](report/main.tex), references and figures in `report/`.
- [Mission demonstration video](video/edit/G22_hazard_mission_1x.mp4), Full HD, 5 min 19 s. The full mission plays continuously at 1× simulation time, with the overview, map and robot points of view.

Optional front cameras on the UGVs (`video_camera:=true`) were used for recording;
the mission does not use them. Raw recordings, editing projects, generated graphics
and temporary review files are excluded from Git.

The map in `maps/` is generated from the world with `scripts/make_map.py`.
The decor (`models/g22_decor/`) is generated with `scripts/make_decor.py`.
The labelled barrels (`models/barrel/`) are generated with `scripts/make_barrels.py`.

## How the mission works

The UAV follows a lawnmower route. Camera signs request an inspection: fly to
the indicated location, average five fresh sensor readings, and publish a
candidate only when the corresponding alarm threshold is exceeded. A sign is
a clue, not evidence of a leak. Targets beyond the survey boundary are checked
at the nearest point inside that boundary.

A sensor alarm without a camera sign starts a short compass search: measure
the centre and its four neighbours, then repeat once if the readings improve.
There are at most nine probes and a 25 s limit for the whole inspection,
including movement. A matching camera sign replaces the blind search with a
direct inspection. The UAV then resumes its route, returns home and lands.
Only readings, camera detections and navigation poses drive these decisions;
the UAV does not know the field formula, background or source locations.
Alarm thresholds are operational settings.

The coordinator sends the nearest free UGV whose Nav2 server is available,
using straight-line distance. Previous tasks do not affect this choice. Failed
navigation or missing ground measurements leave the candidate unconfirmed and
try the other robot. A ground check needs a successful arrival and at least
three fresh readings at the goal.

Candidates and ground checks use `Detection` on `/hazards/detections` and
`/hazards/confirmations`. Camera signs use `Sign` on `/signs`; they request UAV
inspections and are displayed in RViz. The old duplicate `Hazard` message
and `/hazards/candidates` topic have been removed. When upgrading an older
workspace, rebuild the interfaces in clean build/install directories to avoid
stale generated message bindings.

Candidates indicate a measured hazardous area, not the exact position of a
leak. A short search trades source precision for mission time; camera signs
can also sit beside a source. Noise and nearby sources limit precision, and
same-type candidates within `min_separation` (6 m) are grouped together.
The CSV logger reads reference sources only to record errors and never guides
either robot.

The simulated UGV sensors evaluate the field at their actual Gazebo positions,
while the positions attached to their readings come from AMCL. A localisation
error therefore affects the reported location, not the physical measurement.
Only the sensor simulator subscribes to each UGV's `ground_truth` topic.
The UAV uses Gazebo odometry as idealised localisation.

RViz colours ground cells from ground readings only. Blue means a low recorded
reading, yellow caution and red a high reading. Unvisited cells are unassessed;
a low airborne reading does not establish that the ground is safe.

Both UGVs use `config/nav2.yaml`. The simulation launch supplies their separate
namespaces and spawn/AMCL poses. It starts localisation, planning, control and
obstacle avoidance; docking, route graphs, waypoint following and the unused
path-smoothing server are omitted. Obstacle layers are 2D, matching the LiDAR.
AMCL uses 180 of the scanner's 360 beams and updates after 5 cm or 0.05 rad of
motion; the inherited 25 cm interval allowed large localisation errors to build
up on these slow robots.

## Packages

- `g22_hazard_bringup`: world, robot models, map, Nav2 config and the simulation launch
- `g22_hazard_interfaces`: our messages
- `g22_hazard_mission`: our nodes (`hazard_sensor`, `uav_survey`, `coordinator`,
  `hazard_viz`, `mission_logger`, `sign_detector`)

The hidden sources are in `g22_hazard_mission/config/hazards.yaml`. Only the sensor
simulation, the metrics logger and `show_truth` read that file.

## Reused files

The upstream files were committed before adaptation. The shared `nav2.yaml` is
derived from the two original multi-robot parameter files.

| File | Original | Source | Licence |
|---|---|---|---|
| `worlds/hazard_zone.sdf` | `sensor_tutorial.sdf` | Gazebo Harmonic, Sensors tutorial: https://gazebosim.org/docs/harmonic/sensors/ (https://github.com/gazebosim/docs, `harmonic/tutorials/sensors/`) | CC BY 4.0 |
| `models/ugv/ugv.sdf.xacro` | `vehicle_blue` in `sensor_tutorial.sdf` | same tutorial (also "Building your own robot" and "Moving the robot") | CC BY 4.0 |
| `models/x3_uav/` | X3 UAV, version 4 (meshes replaced by simple shapes) | Gazebo Fuel: https://app.gazebosim.org/OpenRobotics/fuel/models/X3%20UAV | CC BY 4.0 |
| `models/x3_uav/model.sdf` (plugins) | `multicopter_velocity_control.sdf` | gz-sim 8 example worlds: https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds | Apache 2.0 |
| `launch/sim.launch.py` | `unique_multi_tb3_simulation_launch.py` | nav2_bringup 1.3.13 (Jazzy): https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup | Apache 2.0 |
| `config/nav2.yaml` | `nav2_multirobot_params_1.yaml`, `nav2_multirobot_params_2.yaml` | nav2_bringup 1.3.13 | Apache 2.0 |
| `models/jersey_barrier/`, `pickup/`, `cinder_block/`, `dumpster/`, `euro_pallet/`, `cardboard_box/`, `construction_cone/`, `fire_hydrant/`, `reactor/`, `water_tower/`, `telephone_pole/`, `collapsed_industrial/` | same model folders (texture paths in the `.dae` files fixed so Gazebo Harmonic finds them) | Gazebo classic model database: https://github.com/osrf/gazebo_models | CC BY 3.0 |

Paths above are inside `ros2_ws/src/g22_hazard_bringup/`. Everything in
`g22_hazard_interfaces` and `g22_hazard_mission` is ours.

The decor textures and meshes in `models/g22_decor/` are ours: they are generated by
`scripts/make_decor.py` (procedural textures, meshes the same size as the boxes and
cylinders of the world, so the collisions and the Nav2 map stay simple).

## Moodle

- Project proposal (`RSDIS_Proposal_MultiRobot.pdf`): https://moodle.isep.ipp.pt/mod/resource/view.php?id=85865
- Report template (`RSDIS 2023 IEEE Template.zip`, unzipped in `report/`): https://moodle.isep.ipp.pt/mod/resource/view.php?id=65985
- Submission: https://moodle.isep.ipp.pt/mod/assign/view.php?id=65987
