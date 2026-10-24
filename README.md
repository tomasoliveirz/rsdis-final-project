# RSDIS Final Project - Group 22

Multi-robot simulation with ROS 2 and Gazebo for RSDIS (ISEP, 2026/27).

## Base files

This commit only has the files we start from, exactly as they were downloaded.

| File | Source | Licence |
|---|---|---|
| `worlds/sensor_tutorial.sdf` | Gazebo Harmonic, Sensors tutorial: https://gazebosim.org/docs/harmonic/sensors/ (https://github.com/gazebosim/docs, `harmonic/tutorials/sensors/`) | CC BY 4.0 |
| `worlds/multicopter_velocity_control.sdf` | gz-sim 8 example worlds: https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds | Apache 2.0 |
| `models/x3_uav/` | X3 UAV, version 4, Gazebo Fuel: https://app.gazebosim.org/OpenRobotics/fuel/models/X3%20UAV (only `model.config` and `model.sdf`) | CC BY 4.0 |
| `launch/unique_multi_tb3_simulation_launch.py` | nav2_bringup 1.3.13 (Jazzy): https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup | Apache 2.0 |
| `config/nav2_multirobot_params_1.yaml`, `config/nav2_multirobot_params_2.yaml` | nav2_bringup 1.3.13 | Apache 2.0 |

Paths above are inside `ros2_ws/src/g22_hazard_bringup/`.

## Moodle

- Project proposal (`RSDIS_Proposal_MultiRobot.pdf`): https://moodle.isep.ipp.pt/mod/resource/view.php?id=85865
- Report template (`RSDIS 2023 IEEE Template.zip`, unzipped in `report/`): https://moodle.isep.ipp.pt/mod/resource/view.php?id=65985
- Submission: https://moodle.isep.ipp.pt/mod/assign/view.php?id=65987
