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
from planning.explorer import FrontierExplorer  # noqa: E402
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

# Side ToF beam angles (rad) relative to the rover heading: -70 / +70 / +90 degrees.
# Nominal; replace with the MEASURED mounted angles. The list order must match the Qwiic
# chain order (index 0 = the module physically at the first angle). See bridge/AGENT.MD.
TOF_ANGLES = [-1.22, 1.22, 1.57]
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
        self.explorer = FrontierExplorer()
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
        self._explore_ticks = 0  # used to re-plan exploration periodically

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

        # 3. Mapping: assemble ONE scan (rover-frame rays) and fuse it in a single update.
        #    Per AGENT.MD every sensor contributes (angle_offset, range_m) rays.
        telemetry = self.car.read_telemetry()
        scan: list[tuple[float, float]] = []
        # PRIMARY sensing: the phone's full depth image projected (6-DoF) and sliced into a
        # 2D obstacle scan. This is what actually fills the map as the rover drives.
        scan += self.slam.last_depth_scan()
        # (Step 2) Side ToF: the 3 Modulino Distance beams at fixed angles (still stubbed).
        # scan += [(a, r) for a, r in zip(TOF_ANGLES, self.imu.read_distances()) if r is not None]
        # Forward ultrasonic as one extra ray - catches glass / dark surfaces the LiDAR misses.
        if telemetry and telemetry.ultrasonic_distance is not None and telemetry.ultrasonic_distance >= 0:
            scan.append((0.0, telemetry.ultrasonic_distance))
        if scan:
            self.grid.update(pose, scan)

        # 6. Broadcast the live map. The route home goes in return_path ONLY while
        #    returning (empty while exploring), so the viewer can tell the two apart.
        return_path = self.planner.current_path() if self.returning else []
        home = {"x": self.start_pose.x, "y": self.start_pose.y} if self.start_pose else None
        self.server.publish(self.grid.to_map_update(pose, return_path, start=home))

        # Return-home failsafe: if the battery-time budget is reached and nobody has
        # commanded a return yet, start it automatically while charge remains.
        if (
            not self.returning
            and self.mission_start is not None
            and time.time() - self.mission_start >= FAILSAFE_RETURN_S
        ):
            print(f"[failsafe] {FAILSAFE_RETURN_S}s battery-time budget reached; returning home")
            self.request_return()

        # 4 / 5. Autonomous behavior: explore the unknown, then drive home.
        if self.returning:
            cmd = self.planner.next_command(pose)
        else:
            self.driven_path.append(pose)
            cmd = self.autonomous_explore(pose)
        if cmd is not None:
            self.car.send_drive(cmd)

    def autonomous_explore(self, pose: Pose) -> DriveCommand | None:
        """Drive the rover toward the next frontier; return home when exploration is done.

        Re-plans when the current frontier path is finished, or periodically so it adapts
        to the map it is revealing as it drives. When no reachable frontiers remain, hands
        off to the return planner.
        """
        self._explore_ticks += 1
        need_target = not self.planner.current_path() or self.planner.finished()
        if need_target or self._explore_ticks % 15 == 0:
            path = self.explorer.next_path(self.grid, pose)
            if path is None:
                print("[explore] no reachable frontiers left; returning home")
                self.request_return()
                return self.planner.next_command(pose)
            self.planner.set_path(path)
        return self.planner.next_command(pose)

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
