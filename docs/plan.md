# Plan

## Idea

An industrial / post-apocalyptic area in Gazebo with zones contaminated by gas and
radiation, explored by one UAV and two ground robots.

The UAV flies a waypoint route over the area taking readings. When it measures something
abnormal it does a small search around that point to get a better estimate of where the
source is, and sends that position to a coordinator. The coordinator picks the closest of
the two ground robots, which drives there with Nav2, takes a measurement on the ground and
confirms the hazard.

Some barrels have a gas or radiation sign on them. A sign only means there might be a
leak, so the UAV camera picks up the signs with OpenCV and the UAV checks the readings
around each one.

At the end RViz shows the safe and dangerous zones and the detections.

## What we reuse

- World: the Gazebo Sensors tutorial world, turned into an industrial yard
- UAV: X3 from Gazebo Fuel, with the velocity controller from the gz-sim multicopter example
- Ground robots: the robot from the Gazebo tutorials (box, two wheels, caster, lidar), made smaller
- Navigation: the Nav2 multi-robot launch and parameters from `nav2_bringup`
- Decor: barriers, car wrecks, pallets, cooling tower etc. from the Gazebo classic model database

## What we write

- simulated gas and radiation sensors
- UAV route and local search
- sign detection on the UAV camera images (OpenCV)
- coordinator that sends the closest ground robot
- hazard markers in RViz

## To do

- [x] world with the UAV and two ground robots running Nav2
- [x] gas and radiation sensors
- [x] UAV route and local search
- [x] signs on the barrels and OpenCV sign detection
- [x] coordinator
- [x] RViz markers (zones, robots, estimates and confirmations)
- [x] whole mission from one launch file (`all.launch.py`)
- [x] metrics logger (CSV per run in `~/g22_runs`)
- [ ] video
- [ ] report

Deadline: 8 November 2026 (report of up to 6 pages, ROS packages and video).
