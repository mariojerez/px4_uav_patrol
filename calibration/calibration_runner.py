"""Sequential ExecutePatrol orchestrator for the energy-cost calibration
suite.

Reads a `calibration_manifest.json` produced by `gen_calibration_missions.gen_all`
and drives the existing patrol_mission action server through every mission
in order. Each mission is sent with `auto_arm=True` so the patrol_mission
node arms the FMU and switches into the registered Patrol custom mode
without the operator touching QGC.

Recovery policy: each mission is attempted twice. On a second failure the
suite aborts and `run_manifest.json` is written with `status="aborted"`
plus the mission that failed. No retry on cancelled goals (operator
cancellation is honored).

Entrypoint: a single `ros2 run` / `ros2 launch` invocation. Keeps the node
alive as long as the suite is running; exits after writing `run_manifest.json`.
"""

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from shepherding_msgs.action import ExecutePatrol
from px4_msgs.msg import VehicleStatus


class CalibrationRunner(Node):
    """Sends one ExecutePatrol goal per mission, waits for each to complete."""

    def __init__(
        self,
        manifest_path: str,
        output_dir: str,
        action_server: str = "/execute_patrol",
        wait_for_fmu_s: float = 60.0,
        retry_per_mission: int = 1,
        run_hover: bool = True,
        run_straight_line: bool = True,
        run_turn: bool = True,
        n_trials_override: Optional[int] = None,
    ):
        super().__init__("calibration_runner")
        self._manifest_path = Path(manifest_path)
        self._output_dir = Path(output_dir)
        self._wait_for_fmu_s = wait_for_fmu_s
        self._retry_per_mission = retry_per_mission
        self._run_hover = run_hover
        self._run_straight_line = run_straight_line
        self._run_turn = run_turn
        self._n_trials_override = n_trials_override

        self._action_client = ActionClient(self, ExecutePatrol, action_server)
        self._fmu_ready = False
        self._fmu_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self._fmu_status_cb,
            qos_profile_sensor_data,
        )
        self._cancelled = False

        self._results: List[Dict] = []

    def _fmu_status_cb(self, msg: VehicleStatus):
        # Only flip ready=True once pre-flight checks pass. PX4 SITL takes
        # ~5-10 s to settle.
        if not self._fmu_ready and msg.pre_flight_checks_pass:
            self.get_logger().info("FMU pre-flight checks pass — runner ready")
            self._fmu_ready = True

    def cancel(self, *_):
        self.get_logger().warn("Cancellation requested; will exit after current mission")
        self._cancelled = True

    def wait_for_fmu(self) -> bool:
        deadline = time.time() + self._wait_for_fmu_s
        while not self._fmu_ready and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not self._fmu_ready:
            self.get_logger().error(
                f"FMU did not become ready within {self._wait_for_fmu_s} s"
            )
            return False
        return True

    def wait_for_action_server(self, timeout_s: float = 30.0) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().error(
                f"Action server '{self._action_client._action_name}' not available"
            )
            return False
        return True

    def run_mission(self, mission_path: str, meta_path: str) -> Dict:
        """Send one ExecutePatrol goal; block until the result arrives.

        Returns a dict with status (success/failure/error/cancelled),
        energy_joules, distance_meters, and a duration_s wall-clock.
        """
        goal = ExecutePatrol.Goal()
        goal.mission_file = str(mission_path)
        goal.per_leg_meta_file = str(meta_path)
        goal.auto_arm = True

        t0 = time.time()
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {
                "status": "rejected",
                "error": "Action goal not accepted",
                "duration_s": time.time() - t0,
            }

        result_future = goal_handle.get_result_async()
        # Spin until completion, but allow cancellation to interrupt.
        while not result_future.done() and not self._cancelled:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._cancelled and not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            return {
                "status": "cancelled",
                "duration_s": time.time() - t0,
            }

        result_msg = result_future.result()
        if result_msg is None:
            return {
                "status": "error",
                "error": "Result was None",
                "duration_s": time.time() - t0,
            }

        # rclpy.action result code: 4=succeeded, 5=cancelled, 6=aborted
        status_code = result_msg.status
        result = result_msg.result
        return {
            "status": "success" if status_code == 4 and result.success else "failure",
            "status_code": int(status_code),
            "energy_joules": float(result.energy_joules),
            "distance_meters": float(result.distance_meters),
            "duration_s": time.time() - t0,
        }

    def run_suite(self) -> Dict:
        if not self._manifest_path.exists():
            return {"status": "error", "reason": f"manifest not found: {self._manifest_path}"}
        with self._manifest_path.open() as f:
            manifest = json.load(f)
        missions = manifest["missions"]

        # Filter by experiment toggles
        def keep(entry):
            exp = entry["experiment"]
            return (
                (exp == "hover" and self._run_hover)
                or (exp == "straight_line" and self._run_straight_line)
                or (exp == "turn" and self._run_turn)
            )
        missions = [m for m in missions if keep(m)]
        self.get_logger().info(
            f"Calibration suite: {len(missions)} missions queued"
        )

        for i, entry in enumerate(missions):
            if self._cancelled:
                self.get_logger().warn("Suite cancelled before completion")
                return {
                    "status": "cancelled",
                    "completed": i,
                    "total": len(missions),
                    "results": self._results,
                }

            self.get_logger().info(
                f"[{i + 1}/{len(missions)}] {entry['name']} ({entry['experiment']})"
            )
            attempt = 0
            outcome = None
            while attempt <= self._retry_per_mission:
                outcome = self.run_mission(
                    entry["mission_path"], entry["meta_path"],
                )
                outcome["mission"] = entry["name"]
                outcome["experiment"] = entry["experiment"]
                outcome["attempt"] = attempt + 1
                if outcome["status"] == "success":
                    break
                self.get_logger().warn(
                    f"Mission {entry['name']} attempt {attempt + 1} returned "
                    f"{outcome['status']}; retry={attempt < self._retry_per_mission}"
                )
                attempt += 1

            self._results.append(outcome)
            if outcome["status"] != "success":
                self.get_logger().error(
                    f"Mission {entry['name']} failed after "
                    f"{self._retry_per_mission + 1} attempts. Aborting suite."
                )
                return {
                    "status": "aborted",
                    "failed_mission": entry["name"],
                    "results": self._results,
                }

        return {
            "status": "success",
            "completed": len(missions),
            "total": len(missions),
            "results": self._results,
        }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the energy-cost calibration suite via ExecutePatrol",
    )
    parser.add_argument("--manifest", required=True,
                        help="Path to calibration_manifest.json")
    parser.add_argument("--output-dir", required=True,
                        help="Where to write run_manifest.json")
    parser.add_argument("--action-server",
                        default="/execute_patrol")
    parser.add_argument("--wait-for-fmu-s", type=float, default=60.0)
    parser.add_argument("--no-hover", action="store_true")
    parser.add_argument("--no-straight-line", action="store_true")
    parser.add_argument("--no-turn", action="store_true")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    runner = CalibrationRunner(
        manifest_path=args.manifest,
        output_dir=args.output_dir,
        action_server=args.action_server,
        wait_for_fmu_s=args.wait_for_fmu_s,
        run_hover=not args.no_hover,
        run_straight_line=not args.no_straight_line,
        run_turn=not args.no_turn,
    )

    # Operator can SIGINT to gracefully cancel between missions.
    signal.signal(signal.SIGINT, runner.cancel)
    signal.signal(signal.SIGTERM, runner.cancel)

    rc = 0
    try:
        if not runner.wait_for_action_server():
            rc = 2
        elif not runner.wait_for_fmu():
            rc = 3
        else:
            outcome = runner.run_suite()
            run_manifest_path = Path(args.output_dir) / "run_manifest.json"
            run_manifest_path.parent.mkdir(parents=True, exist_ok=True)
            with run_manifest_path.open("w") as f:
                json.dump(outcome, f, indent=2)
            runner.get_logger().info(
                f"Wrote {run_manifest_path} (status={outcome['status']})"
            )
            rc = 0 if outcome["status"] == "success" else 1
    finally:
        runner.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
