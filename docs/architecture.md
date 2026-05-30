# Architecture

## Goal

A human drives the rover into an unknown space. The rover builds a metric map on the edge,
on the rover itself, with no network. On a single command it plans a route over that map
and drives itself back to the start. A phone on the rover's own Wi-Fi watches the map fill
in live. Dense 3D reconstruction is a later bonus, only when a network link exists.

## The edge / cloud split

Everything mission-critical is on the edge. There is no cloud dependency for driving,
mapping, planning, or live viewing.

- Edge (always): teleop, SLAM, mapping, return planning, the web server, and the Wi-Fi
  access point all run on the UNO Q Dragonwing. The phone connects directly to the
  rover's AP.
- Cloud (optional bonus, later): dense 3D reconstruction. Only attempted when an external
  network link exists. It never blocks the return mission. Not part of this scaffold.

## Hardware

| Part | Role |
| --- | --- |
| Elegoo Smart Robot Car V4 | Locomotion. Onboard Arduino UNO R3 + TB6612 motor driver stays as the dedicated motor controller. Servo-mounted ultrasonic and line sensors. No wheel encoders. |
| Arduino UNO Q | The brain. Qualcomm Dragonwing runs Debian Linux (Python): SLAM, mapping, planning, web server, Wi-Fi AP. Onboard STM32 / RPC "Bridge" handles real-time sensor reads. |
| Modulino Movement (IMU) | Visual-inertial odometry source. Over Qwiic / I2C. |
| Modulino Distance (ToF) | Range sensing. Over Qwiic / I2C. |
| Logitech USB webcam (mono) | The single camera for SLAM. |

Because there are no wheel encoders, odometry is visual-inertial: the mono camera plus the
IMU. This is why the SLAM frontend ingests both frames and `ImuSample` data.

## Components and data flow

```
  webcam (USB) ----frame----+
                            v
  Modulino IMU --ImuSample-->  [ slam/slam_frontend.py ]
                                       | Pose
                                       v
  Modulino ToF --range-->     [ mapping/occupancy_grid.py ]
                                       | grid + Pose
                          +------------+------------+
                          |                         |
                          v                         v
              [ planning/return_planner.py ]   [ server/app.py ]
                          | return_path              | MapUpdate (websocket)
                          +-----------> MapUpdate <---+
                                                      v
                                          phone browser (visualization)

  [ bridge/car_link.py ] <--DriveCommand / CarTelemetry--> Elegoo UNO R3 (car-firmware)
  [ bridge/modulino_io.py ] <--I2C--> Modulino IMU + ToF
```

### Run loop (navigation/main.py orchestrator)

1. Read one IMU sample and one camera frame.
2. `slam_frontend.process(frame, imu)` returns a `Pose` and, on keyframes, map structure.
3. `occupancy_grid.update(pose, range_reading)` folds new range data into the grid.
4. While teleoperating, forward the human `DriveCommand` to `car_link`.
5. On the return command, `return_planner.plan(grid, start, current_pose)` produces a
   `return_path`; the orchestrator follows it by emitting `DriveCommand`s.
6. `server.app` broadcasts a `MapUpdate` (grid + pose + path) to connected phones.

### Failure and fallback

- If SLAM loses tracking or the grid is too sparse to plan, `return_planner` falls back to
  reversing the logged drive path (a breadcrumb list of poses recorded during teleop).
- The car firmware brakes on loss of serial commands (watchdog) so a brain crash does not
  leave the motors running.

## Message contract

All cross-component messages are defined once in
[message-schemas.md](message-schemas.md) and mirrored in
[../shared/schemas/](../shared/schemas/). The five messages are `DriveCommand`,
`CarTelemetry`, `Pose`, `ImuSample`, and `MapUpdate` (which embeds `Pose` and a list of
waypoints).
