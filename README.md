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
ros2 launch g22_hazard_mission sensors.launch.py
```

Starts the simulated gas/radiation sensor on each robot, the hazard map and RViz. Add
`show_truth:=true` to also draw the real sources.

The map in `maps/` is generated from the world with `scripts/make_map.py`.

## Packages

- `g22_hazard_bringup`: world, robot models, map, Nav2 config and the simulation launch
- `g22_hazard_interfaces`: our messages
- `g22_hazard_mission`: our nodes (`hazard_sensor`, `hazard_viz`)

The hidden sources are in `g22_hazard_mission/config/hazards.yaml`. Only the sensor
simulation reads that file.

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

Paths above are inside `ros2_ws/src/g22_hazard_bringup/`. Everything in
`g22_hazard_interfaces` and `g22_hazard_mission` is ours.

## Moodle

- Project proposal (`RSDIS_Proposal_MultiRobot.pdf`): https://moodle.isep.ipp.pt/mod/resource/view.php?id=85865
- Report template (`RSDIS 2023 IEEE Template.zip`, unzipped in `report/`): https://moodle.isep.ipp.pt/mod/resource/view.php?id=65985
- Submission: https://moodle.isep.ipp.pt/mod/assign/view.php?id=65987
