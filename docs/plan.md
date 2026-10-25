# Plan

## Idea

An industrial / post-apocalyptic area in Gazebo with zones contaminated by gas and
radiation, explored by one UAV and two ground robots.

The UAV flies a waypoint route over the area taking readings. When it measures something
abnormal it does a small search around that point to get a better estimate of where the
source is, and sends that position to a coordinator. The coordinator picks the closest of
the two ground robots, which drives there with Nav2, takes a measurement on the ground and
confirms the hazard.

At the end RViz shows the safe and dangerous zones and the detections.

## What we reuse

- World: the Gazebo Sensors tutorial world, turned into an industrial yard
- UAV: X3 from Gazebo Fuel, with the velocity controller from the gz-sim multicopter example
- Ground robots: the robot from the Gazebo tutorials (box, two wheels, caster, lidar), made smaller
- Navigation: the Nav2 multi-robot launch and parameters from `nav2_bringup`

## What we write

- simulated gas and radiation sensors
- UAV route and local search
- coordinator that sends the closest ground robot
- hazard markers in RViz

## To do

- [x] world with the UAV and two ground robots running Nav2
- [x] gas and radiation sensors
- [ ] UAV route and local search
- [ ] coordinator
- [x] RViz markers (zones and robots, detections still to do)
- [ ] whole mission from one launch file
- [ ] video
- [ ] report

Deadline: 8 November 2026 (report of up to 6 pages, ROS packages and video).
