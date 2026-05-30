"""Recon Rover - navigation orchestrator.

Wires the stubbed modules into the run loop described in docs/architecture.md. This file
defines the *shape* of the mission so the team can fill in each module against a stable
interface. It does not implement the hard algorithms.

Run loop, per tick:
  1. Read one IMU sample (modulino_io) and one camera frame.
  2. slam.process(frame, imu) -> Pose.
  3. mapping.update(pose, range_reading) folds range data into the occupancy grid.
  4. While teleoperating, forward the human DriveCommand to the car (car_link).
  5. On the return command, planning.plan(grid, start, pose) -> return_path, then follow it.
  6. server broadcasts a MapUpdate (grid + pose + path) to connected phones.

This is a scaffold: every call below lands in a stub with a TODO.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Make the shared message contract importable.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "shared"))

from schemas.schemas import DriveCommand, Pose  # noqa: E402  (after sys.path tweak)

from bridge.car_link import CarLink  # noqa: E402
from bridge.modulino_io import ModulinoIO  # noqa: E402
from mapping.occupancy_grid import OccupancyGrid  # noqa: E402
from perception.detector import Detector  # noqa: E402
from planning.return_planner import ReturnPlanner  # noqa: E402
from server.app import MapServer  # noqa: E402
from slam.slam_frontend import SlamFrontend  # noqa: E402


# --- Return-home failsafe -------------------------------------------------------------
# Automatically trigger the return before the battery dies, so the rover always keeps
# enough charge to drive itself home even if no operator ever presses "Return". Sized from
# the Elegoo Smart Robot Car V4 power budget below.
#
# THESE ARE CONSERVATIVE PLANNING ESTIMATES, NOT GUARANTEES. Measure a real run (drive the
# fully-built rover continuously and time it to shutdown) and set MISSION_BUDGET_S to what
# you actually observe. The numbers here are the starting point:
#
#   Pack:        2x 18650 Li-ion in series -> 7.4 V nominal, ~2000 mAh (~11-12 Wh usable
#                after ~80% depth-of-discharge; cheaper included cells can be far less).
#   Car only:    2 gear motors + ultrasonic/servo + line sensors + UNO R3 ~= 5 W under
#                light continuous driving -> ~90 min on the pack.
#   Car + brain: add the UNO Q doing SLAM + USB camera + Wi-Fi hotspot, ~3-4 W, IF it is
#                powered from the SAME pack -> total ~8-9 W -> ~45-60 min.
#
# The dominant variable is how the UNO Q is powered:
#   - Brain on the car pack (assumed here):     MISSION_BUDGET_S ~= 45 min.
#   - Brain on its own battery / power bank:    bump MISSION_BUDGET_S toward ~90 min.
#
# We plan for the loaded case and fire at ~65% of the budget, leaving a healthy margin for
# the return drive itself plus a safety reserve.
MISSION_BUDGET_S = 45 * 60  # estimated usable runtime with the brain on the car pack
FAILSAFE_RETURN_S = int(0.65 * MISSION_BUDGET_S)  # ~29 min; auto-return with margin left
# TODO: upgrade to a true low-battery trigger once the car reports battery_voltage in
#       CarTelemetry (a voltage divider on a spare analog pin). Voltage sags under motor
#       load, so filter it; a timeout is the robust backstop regardless.


class Orchestrator:
    """Owns the modules and drives the per-tick run loop.

    The orchestrator is intentionally thin: it sequences calls and routes the message
    objects between modules. All real work happens inside the modules.
    """

    def __init__(self) -> None:
        self.car = CarLink()
        self.imu = ModulinoIO()
        self.slam = SlamFrontend()
        self.grid = OccupancyGrid()
        self.planner = ReturnPlanner()
        self.detector = Detector()
        self.server = MapServer()
        # The phone's "return home" button routes here, the same path as the failsafe.
        self.server.on_return_request = self.request_return

        # Breadcrumb trail of poses recorded during teleop. The planner uses this as the
        # reverse-path fallback when the grid is too sparse for A*.
        self.driven_path: list[Pose] = []
        self.start_pose: Pose | None = None
        self.returning = False
        self.mission_start: float | None = None  # set at power-on for the failsafe clock

    def setup(self) -> None:
        """Open hardware links. TODO: open the camera device here too."""
        self.car.connect()
        self.imu.connect()
        self.slam.connect()  # open the phone (Record3D) perception stream
        self.server.run_in_thread()  # serve the viz + open the map websocket for phones
        self.mission_start = time.time()  # start the battery-time failsafe clock
        # TODO: open the Logitech USB webcam (cv2.VideoCapture) and store the handle.

    def read_frame(self):
        """Grab one mono camera frame.

        Returns: an image array, or None if no frame is available.
        TODO: read from the cv2.VideoCapture opened in setup().
        """
        return None

    def tick(self) -> None:
        """Run one iteration of the mission loop."""
        frame = self.read_frame()
        imu_sample = self.imu.read_imu()

        # 2. SLAM: frame + IMU -> Pose.
        pose = self.slam.process(frame, imu_sample)
        if pose is None:
            return  # tracking not yet available

        if self.start_pose is None:
            self.start_pose = pose  # remember where "home" is

        # 3. Mapping: prefer the phone's dense depth; fall back to the ultrasonic range.
        telemetry = self.car.read_telemetry()
        depth = self.slam.last_depth()
        if depth is not None:
            self.grid.update_from_depth(pose, depth[0], depth[1])
        else:
            range_m = telemetry.ultrasonic_distance if telemetry else None
            self.grid.update(pose, range_m)

        # 6. Broadcast the live map (grid + pose + planned path) to any connected phones.
        return_path = self.planner.current_path() if self.returning else []
        self.server.publish(self.grid.to_map_update(pose, return_path))

        # Return-home failsafe: if the battery-time budget is reached and nobody has
        # commanded a return yet, start it automatically while charge remains.
        if (
            not self.returning
            and self.mission_start is not None
            and time.time() - self.mission_start >= FAILSAFE_RETURN_S
        ):
            print(f"[failsafe] {FAILSAFE_RETURN_S}s battery-time budget reached; returning home")
            self.request_return()

        # 4 / 5. Teleop vs return.
        if self.returning:
            cmd = self.planner.next_command(pose)
        else:
            self.driven_path.append(pose)
            cmd = self.teleop_command()
        if cmd is not None:
            self.car.send_drive(cmd)

    def teleop_command(self) -> DriveCommand | None:
        """Get the latest human drive command.

        Returns: a DriveCommand, or None if no fresh input.
        TODO: source this from the web UI / gamepad input path.
        """
        return None

    def request_return(self) -> None:
        """Handle the single 'go home' command: plan a route and start following it."""
        assert self.start_pose is not None, "no start pose recorded yet"
        path = self.planner.plan(self.grid, self.start_pose, self.driven_path)
        self.planner.set_path(path)
        self.returning = True

    def run(self, hz: float = 20.0) -> None:
        """Main loop. Ticks at the requested rate until interrupted."""
        self.setup()
        period = 1.0 / hz
        try:
            while True:
                self.tick()
                time.sleep(period)
        except KeyboardInterrupt:
            self.car.send_drive(DriveCommand(0.0, 0.0, stop=1))
            print("stopped")


if __name__ == "__main__":
    Orchestrator().run()
