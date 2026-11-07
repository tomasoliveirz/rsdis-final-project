# RSDIS Final Project - Group 22

Multi-robot simulation with ROS 2 and Gazebo for RSDIS (ISEP, 2026/27): one UAV and two
ground robots in an industrial area with gas and radiation sources.

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

The map in `maps/` is generated from the world with `scripts/make_map.py`.
The decor (`models/g22_decor/`) is generated with `scripts/make_decor.py`.
The labelled barrels (`models/barrel/`) are generated with `scripts/make_barrels.py`.

## How the mission works

The UAV follows a lawnmower route. A gas or radiation reading over a configured
alarm threshold starts a local search: measure the centre and its neighbours,
move towards higher averaged readings, then reduce the step until the search
finishes. Only sensor readings and robot positions drive this search. The UAV
does not know the field formula, background values or source positions.
The alarm thresholds are operational settings, not the simulated background.

The coordinator sends the nearest free UGV whose Nav2 server is available,
using straight-line distance. Previous tasks do not affect this choice. Failed
navigation or missing ground measurements leave the candidate unconfirmed and
try the other robot. A ground check needs a successful arrival and at least
three fresh readings at the goal.

Candidates and ground checks use `Detection` on `/hazards/detections` and
`/hazards/confirmations`. Camera signs use `Sign` on `/signs`; they are displayed
in RViz but do not yet change the UAV route. The old duplicate `Hazard` message
and `/hazards/candidates` topic have been removed. When upgrading an older
workspace, rebuild the interfaces in clean build/install directories to avoid
stale generated message bindings.

The local search finds a measured peak, not an exact fitted source position.
Noise, broad peaks and nearby sources limit its precision. The CSV logger reads
the reference sources only to record errors; it does not guide either robot.

## Packages

- `g22_hazard_bringup`: world, robot models, map, Nav2 config and the simulation launch
- `g22_hazard_interfaces`: our messages
- `g22_hazard_mission`: our nodes (`hazard_sensor`, `uav_survey`, `coordinator`,
  `hazard_viz`, `mission_logger`, `sign_detector`)

The hidden sources are in `g22_hazard_mission/config/hazards.yaml`. Only the sensor
simulation, the metrics logger and `show_truth` read that file.

## Reused files

The first commit has these files exactly as they were downloaded. Changes come in later
commits.

| File | Original | Source | Licence |
|---|---|---|---|
| `worlds/hazard_zone.sdf` | `sensor_tutorial.sdf` | Gazebo Harmonic, Sensors tutorial: https://gazebosim.org/docs/harmonic/sensors/ (https://github.com/gazebosim/docs, `harmonic/tutorials/sensors/`) | CC BY 4.0 |
| `models/ugv/ugv.sdf.xacro` | `vehicle_blue` in `sensor_tutorial.sdf` | same tutorial (also "Building your own robot" and "Moving the robot") | CC BY 4.0 |
| `models/x3_uav/` | X3 UAV, version 4 (meshes replaced by simple shapes) | Gazebo Fuel: https://app.gazebosim.org/OpenRobotics/fuel/models/X3%20UAV | CC BY 4.0 |
| `models/x3_uav/model.sdf` (plugins) | `multicopter_velocity_control.sdf` | gz-sim 8 example worlds: https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds | Apache 2.0 |
| `launch/sim.launch.py` | `unique_multi_tb3_simulation_launch.py` | nav2_bringup 1.3.13 (Jazzy): https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup | Apache 2.0 |
| `config/nav2_ugv1.yaml`, `config/nav2_ugv2.yaml` | `nav2_multirobot_params_1.yaml`, `nav2_multirobot_params_2.yaml` | nav2_bringup 1.3.13 | Apache 2.0 |
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
