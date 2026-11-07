# Plan

## Idea

An industrial / post-apocalyptic area in Gazebo with zones contaminated by gas and
radiation, explored by one UAV and two ground robots. The yard is 36 × 28 m, with
three gas leaks and two radiation sources.

The UAV flies a waypoint route over the area taking readings. When it measures something
abnormal it does a short search (at most nine probes or 25 s) and sends the
strongest measured position to a coordinator. The coordinator picks the closest available
ground robot, which drives there with Nav2, takes a measurement on the ground and
confirms the hazard.

Some barrels have a gas or radiation sign on them. A sign only means there might be a
leak, so the UAV camera picks up the signs with OpenCV and the UAV checks the readings
around each one.

RViz shows the detections and the ground readings. Low, elevated and high
readings are distinguished; unvisited ground is unassessed, rather than safe.

## Scope of the proposal

The proposal asks for a Gazebo scenario, ROS control/perception modules,
information sharing and complementary robot tasks, plus the report and demo.
The two-UAV team, SLAM/map merging, frontier exploration and auction allocation
are example approaches in the slides. Our scenario uses one UAV, two UGVs,
a known navigation map and nearest-available assignment.

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
- [x] UAV route and local search using sensor readings only
- [x] signs on the barrels and OpenCV sign detection
- [x] use camera signs to request a sensor inspection; a sign alone does not confirm a leak
- [x] nearest available UGV, with no penalty for previous tasks
- [x] reassign failed ground checks without publishing false confirmations
- [x] RViz markers (ground readings, robots, estimates and confirmations)
- [x] whole mission from one launch file (`all.launch.py`)
- [x] metrics logger (CSV per run in `~/g22_runs`)
- [ ] repeat complete Gazebo trials and compare accuracy and mission time
- [ ] video
- [ ] report

Deadline: 8 November 2026 (report of up to 6 pages, ROS packages and video).
