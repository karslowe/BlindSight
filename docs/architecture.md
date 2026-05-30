# Architecture

## Goal

The rover drives itself into an unknown space (autonomous frontier exploration - no human
driving), building a metric map on the edge, on the rover itself, with no network. When the
space is covered it plans a route over that map and drives itself back to the start. A phone
on the rover's own Wi-Fi watches the map fill in live. Dense 3D reconstruction is a later
bonus, only when a network link exists.

## The edge / cloud split

Everything mission-critical is on the edge. There is no cloud dependency for driving,
mapping, planning, or live viewing.

- Edge (always): SLAM runs on the phone (on the rover); mapping, return planning, the web
  server, the Wi-Fi access point, and motor control all run on the single UNO Q (also on
  the rover). No network link is needed for any of it. The viewer phone connects to the
  rover's AP.
- Cloud (optional bonus, later): dense 3D reconstruction. Only attempted when an external
  network link exists. It never blocks the return mission. Not part of this scaffold.

## Hardware

A single UNO Q is the whole rover computer; the phone is the perception sensor. There is no
separate Arduino UNO R3 - the UNO Q's onboard microcontroller drives the car directly.

| Part | Role |
| --- | --- |
| Arduino UNO Q (single board) | The whole rover computer. Dragonwing (Debian / Python) = the brain: mapping, return planning, web server, Wi-Fi AP. Onboard STM32 "Bridge" = the real-time motor controller: drives the Elegoo expansion board's TB6612 and reads the car's ultrasonic + line sensors. |
| iPhone 16 Pro (Record3D) | Perception. Rear camera + LiDAR + IMU; ARKit does metric SLAM on-device and streams pose + depth to the UNO Q over USB (or Wi-Fi). This is the single pose authority. |
| Elegoo Smart Robot Car V4 | Locomotion: 2 motors + TB6612 on the expansion board, servo-mounted ultrasonic, line sensors. No wheel encoders. The UNO Q's STM32 replaces the kit's UNO R3. |
| 3x Modulino Distance (ToF) | Side coverage at -70 / +70 / +90 deg, complementing the phone's forward depth wedge. Over Qwiic / I2C to the UNO Q. |

The mono webcam and the Modulino Movement (IMU) from the original design are dropped: the
phone provides the camera and a fused IMU, and ARKit provides the pose, so the rover does
not implement visual-inertial odometry itself.

Voltage note: the UNO Q's GPIO is 3.3V; the expansion board's 5V sensor outputs (ultrasonic
echo, line sensors) need a level shifter / divider before reaching the UNO Q pins.

## Components and data flow

```
  iPhone 16 Pro (ARKit) --Record3D (USB/Wi-Fi)--> [ slam_frontend wraps it ]
                                                       | Pose + depth wedge (rays)
  3x Modulino ToF --Qwiic/I2C--> (side rays) --------->|
                                                       v
                                      [ mapping/occupancy_grid.py ]  (one scan / tick)
                                                       | grid + Pose
                                  +--------------------+--------------------+
                                  v                                         v
                      [ planning/return_planner.py ]              [ server/app.py ]
                                  | return_path                       | MapUpdate (websocket)
                                  +----------------> MapUpdate <------+
                                                                      v
                                              viewer phone browser (separate Wi-Fi link)

  [ bridge/car_link.py ] <--DriveCommand / CarTelemetry--> UNO Q STM32 (motor MCU)
                                                            drives the shield's TB6612,
                                                            reads ultrasonic + line sensors
```

All of the boxes above run on the single UNO Q except the iPhone (perception) and the
viewer phone (display). `car_link` now talks to the UNO Q's own STM32 over the internal
bridge rather than an external USB cable to a separate board; the `DriveCommand` /
`CarTelemetry` contract is unchanged.

### Run loop (navigation/main.py orchestrator)

1. Read the latest pose + depth frame from the phone (Record3D stream).
2. `slam_frontend.process()` returns the phone's `Pose`; the depth wedge is sampled into rays.
3. Build one scan = the 3 ToF rays + the phone depth-wedge rays, and call
   `occupancy_grid.update(pose, scan)`.
4. While exploring, `explorer.next_path(grid, pose)` routes to the nearest frontier (the
   edge of the mapped area); the orchestrator follows it, emitting `DriveCommand`s to
   `car_link` (to the STM32), revealing more of the space and re-planning as it goes.
5. When no reachable frontiers remain - or the battery-time failsafe fires -
   `return_planner.plan(grid, start, current_pose)` produces a `return_path`, and the
   orchestrator follows it home.
6. `server.app` broadcasts a `MapUpdate` (grid + pose + path) to connected viewer phones.

### Autonomy (no human in the loop)

The rover decides where to go on its own. There are two phases:

- Explore: `planning/explorer.py` finds the frontiers - known-free cells adjacent to
  unknown space - clusters them, and routes to the nearest reachable one with A*
  (`planning/pathfind.py`). Driving there reveals more of the space; it re-plans as the map
  grows. This replaces human teleoperation entirely.
- Return: when no reachable frontiers remain (the space is covered), or the battery-time
  failsafe fires, it switches to `planning/return_planner.py` and drives home.

Both phases share the same A* pathfinder and the same carrot-follow controller; only the
goal differs (a frontier vs. the start). See `server/autonomy_demo.py` for the full loop
running against a synthetic room.

### Failure and fallback

- If ARKit loses tracking or the grid is too sparse to plan, `return_planner` falls back to
  reversing the logged drive path (a breadcrumb list of poses recorded while exploring).
- Stale-frame safety: if the phone stream stalls (USB/Wi-Fi hiccup, ARKit relocalizing,
  thermal throttle), the orchestrator stops/slows rather than acting on an old pose.
- The motor firmware on the STM32 brakes on loss of drive commands (watchdog) so a brain
  hang does not leave the motors running. The car's ultrasonic and line sensors remain
  independent real-time reflexes on the STM32, working even if the phone or brain glitches.

## Message contract

All cross-component messages are defined once in
[message-schemas.md](message-schemas.md) and mirrored in
[../shared/schemas/](../shared/schemas/). The five messages are `DriveCommand`,
`CarTelemetry`, `Pose`, `ImuSample`, and `MapUpdate` (which embeds `Pose` and a list of
waypoints).
