"""Generate PX4 patrol missions + per-leg metadata for the energy-cost
calibration suite.

Three mission families are emitted:

* Hover: one mission, n_trials hovers of `hover_duration_s` each.
* Straight line: one mission per speed in `speeds`. Drone alternates direction
  along a 110 m linear corridor at constant `horizontalVelocity = v`. Each leg
  has a 30 m warmup zone (drone accelerates), a 50 m measurement zone (drone
  at v throughout — snapshot covers this), and a 30 m cooldown zone (next item
  is `hold` so the drone decelerates *after* the snapshot has ended).
* Turn: one mission per (v_g, omega) combination. Each trial: warmup ->
  i -> j -> hold(adaptive duration to allow decel + yaw) -> k -> cooldown.
  Snapshot spans i -> cooldown_end so the drone is at v_g at both snapshot
  boundaries.

Paired with each `<name>.mission.json` is a `<name>.meta.json` that lists per-
leg measurement bounds for the modified EnergyLogger to consume:

    {
      "experiment": "...",
      "csv_path": "<absolute path>",
      "speed_tolerance_mps": 0.3,
      "legs": [
        {"trial_id": "...", "start_idx": int, "end_idx": int,
         "condition": {...}, "expected_distance_m": float,
         "expected_speed_mps": float},
        ...
      ]
    }

The mission JSON validates against the px4-ros2-interface-lib schema at
`/opt/px4_ws/src/px4-ros2-interface-lib/mission/schema.yaml`.
"""

import argparse
import json
import math
import os
from pathlib import Path
from typing import Dict, List, Tuple


METERS_PER_DEG_LAT = 111_320.0

# Default origin: extracted from a shepherd-rl domain config. Each
# /app/shepherding-rl/configs/<farm>.json file has one or more "starting_point"
# zones with [lon, lat] coordinates; the calibration suite uses one of these
# as the local Cartesian origin so that mission waypoints land on the same
# geofence the policy was trained against. Override with --domain-config /
# --starting-point-name, or by setting --origin-lat / --origin-lon directly.
DEFAULT_DOMAIN_CONFIG = "/app/shepherding-rl/configs/lantern_farm.json"
DEFAULT_STARTING_POINT_NAME = "Starting Point 0"

# Hard-coded fallback (PX4 SITL gz_x500 default takeoff). Used only when the
# shepherd-rl submodule is not checked out, so the module still imports.
_FALLBACK_ORIGIN_LAT = 47.397742
_FALLBACK_ORIGIN_LON = 8.545594


def extract_starting_point(
    domain_config_path: str,
    name: str = DEFAULT_STARTING_POINT_NAME,
) -> Tuple[float, float]:
    """Return ``(lat, lon)`` for the named ``starting_point`` zone in a
    shepherd-rl domain config.

    Each config has a top-level ``zones`` array; entries with
    ``type == "starting_point"`` carry a single ``[lon, lat]`` coordinate
    pair. ``name`` selects between multiple starting points (e.g.
    ``"Starting Point 0"`` vs ``"Starting Point 1"`` in lantern_farm.json).
    For configs with a single starting point (umn_golf.json) ``name`` may be
    omitted only if it matches the default; pass the actual name otherwise.
    """
    with open(domain_config_path) as f:
        cfg = json.load(f)
    zones = cfg.get("zones", [])
    starting_points = [z for z in zones if z.get("type") == "starting_point"]
    if not starting_points:
        raise ValueError(
            f"{domain_config_path} has no starting_point zones"
        )
    matched = next((z for z in starting_points if z.get("name") == name), None)
    if matched is None:
        if len(starting_points) == 1:
            matched = starting_points[0]
        else:
            available = [z.get("name") for z in starting_points]
            raise ValueError(
                f"{domain_config_path}: no starting_point named "
                f"{name!r}; available: {available}"
            )
    coords = matched.get("coordinates", [])
    if not coords or len(coords[0]) != 2:
        raise ValueError(
            f"{domain_config_path}: starting_point {name!r} has malformed "
            f"coordinates: {coords!r}"
        )
    lon, lat = coords[0]
    return float(lat), float(lon)


try:
    DEFAULT_ORIGIN_LAT, DEFAULT_ORIGIN_LON = extract_starting_point(
        DEFAULT_DOMAIN_CONFIG, DEFAULT_STARTING_POINT_NAME,
    )
except (FileNotFoundError, ValueError, json.JSONDecodeError):
    DEFAULT_ORIGIN_LAT = _FALLBACK_ORIGIN_LAT
    DEFAULT_ORIGIN_LON = _FALLBACK_ORIGIN_LON

# Defaults are taken from the calibration plan so that callers without an
# override still produce the same mission set.
DEFAULT_N_TRIALS = 7
DEFAULT_ALTITUDE_M = 10.0
DEFAULT_HOVER_DURATION_S = 25.0
DEFAULT_WARMUP_M = 30.0
DEFAULT_STRAIGHT_LINE_M = 50.0
DEFAULT_COOLDOWN_M = 30.0
DEFAULT_TURN_LEG_M = 20.0
DEFAULT_SPEEDS_MPS = (2.0, 4.0, 6.0, 8.0, 10.0)
DEFAULT_TURN_VG_MPS = (2.0, 4.0, 6.0)
DEFAULT_TURN_OMEGA = (0.5, 1.0, 1.5)
DEFAULT_TURN_ANGLES = (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4, math.pi)
DEFAULT_MAX_HEADING_RATE_DEG_S = 60.0  # used for non-turn missions

# PX4 default MPC_ACC_HOR. Keep in sync with the SITL's value or pass via
# arg if your airframe has been retuned.
DEFAULT_MPC_ACC_HOR_MPS2 = 3.0

# Per-trial speed sanity check tolerance in m/s. Trials whose mean speed
# deviates from the target by more than this are excluded from the fit.
DEFAULT_SPEED_TOLERANCE_MPS = 0.3


def local_to_global(
    x_m: float, y_m: float, lat0: float, lon0: float,
) -> Tuple[float, float]:
    """Convert a local Cartesian point (x=east m, y=north m) to (lat, lon)."""
    dlat = y_m / METERS_PER_DEG_LAT
    dlon = x_m / (METERS_PER_DEG_LAT * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def _waypoint_global(
    point_id: str, x_m: float, y_m: float, alt_m: float,
    lat0: float, lon0: float,
) -> Dict:
    lat, lon = local_to_global(x_m, y_m, lat0, lon0)
    return {
        "id": point_id,
        "type": "navigation",
        "navigationType": "waypoint",
        "frame": "global",
        "x": lon,    # mission schema: x = longitude (deg)
        "y": lat,    # mission schema: y = latitude (deg)
        "z": float(alt_m),
    }


def _adaptive_turn_hold_s(
    v_g_mps: float, omega_rad_s: float, delta_theta_rad: float,
    mpc_acc_hor_mps2: float = DEFAULT_MPC_ACC_HOR_MPS2,
    safety_margin_s: float = 1.0,
    floor_s: float = 2.0,
) -> float:
    """Return the hold-duration that lets the drone fully decelerate from v_g
    and yaw by delta_theta at omega before the next waypoint becomes active.

    Decel time: ~v_g / acc (linear deceleration model).
    Yaw time:   delta_theta / omega.
    Plus a safety margin and a floor so even Δθ=0 holds long enough to settle.
    """
    decel_s = v_g_mps / max(mpc_acc_hor_mps2, 1e-6)
    yaw_s = delta_theta_rad / max(omega_rad_s, 1e-6)
    return max(floor_s, decel_s + yaw_s + safety_margin_s)


# ---------------------------------------------------------------------------
# Hover mission
# ---------------------------------------------------------------------------

def gen_hover_mission(
    n_trials: int,
    csv_path: str,
    altitude_m: float = DEFAULT_ALTITUDE_M,
    hover_duration_s: float = DEFAULT_HOVER_DURATION_S,
    origin_lat: float = DEFAULT_ORIGIN_LAT,
    origin_lon: float = DEFAULT_ORIGIN_LON,
    speed_tolerance_mps: float = DEFAULT_SPEED_TOLERANCE_MPS,
) -> Tuple[Dict, Dict]:
    """Build the hover mission + meta.

    Layout (item indices in parens):
        takeoff (0)
        changeSettings (1)
        For trial 0..n-1:
            waypoint hover_anchor_<t>   (2 + 2*t)      <- snapshot start
            hold(hover_duration_s)      (2 + 2*t + 1)
        waypoint hover_anchor_terminal  (2 + 2*n)      <- snapshot end of last trial
        rtl                             (2 + 2*n + 1)

    Each trial's leg is [start_idx = 2 + 2*t, end_idx = 2 + 2*(t+1)].

    horizontalVelocity is set to a small positive value (0.5 m/s) — PX4
    rejects zero — and maxHeadingRate is set to 0 so the drone does not yaw
    during hover (yaw motion would contaminate P_hover).
    """
    items: List[Dict] = [{"type": "takeoff"}]
    items.append({
        "type": "changeSettings",
        "horizontalVelocity": 0.5,
        "maxHeadingRate": 0.0,
    })

    legs = []
    for t in range(n_trials):
        anchor_idx = len(items)
        items.append(_waypoint_global(
            f"hover_anchor_{t}", 0.0, 0.0, altitude_m, origin_lat, origin_lon,
        ))
        items.append({"type": "hold", "duration": float(hover_duration_s)})
        legs.append({
            "start_idx": anchor_idx,
            "end_idx": anchor_idx + 2,  # the next anchor (or terminal)
            "trial_id": f"h{t:02d}",
            "condition": {"experiment": "hover", "trial_index": t,
                          "speed_mps": 0.0},
            "expected_distance_m": 0.0,
            "expected_speed_mps": 0.0,  # 0 disables the speed check
        })

    items.append(_waypoint_global(
        "hover_anchor_terminal", 0.0, 0.0, altitude_m,
        origin_lat, origin_lon,
    ))
    items.append({"type": "rtl"})

    mission = {
        "version": 1,
        "mission": {
            "defaults": {
                "horizontalVelocity": 0.5,
                "verticalVelocity": 0.5,
                "maxHeadingRate": 0.0,
            },
            "items": items,
        },
    }
    meta = {
        "experiment": "hover",
        "csv_path": csv_path,
        "speed_tolerance_mps": speed_tolerance_mps,
        "legs": legs,
    }
    return mission, meta


# ---------------------------------------------------------------------------
# Straight-line missions
# ---------------------------------------------------------------------------

def gen_straight_line_mission(
    speed_mps: float,
    n_trials: int,
    csv_path: str,
    altitude_m: float = DEFAULT_ALTITUDE_M,
    warmup_m: float = DEFAULT_WARMUP_M,
    measure_m: float = DEFAULT_STRAIGHT_LINE_M,
    cooldown_m: float = DEFAULT_COOLDOWN_M,
    max_heading_rate_deg_s: float = DEFAULT_MAX_HEADING_RATE_DEG_S,
    origin_lat: float = DEFAULT_ORIGIN_LAT,
    origin_lon: float = DEFAULT_ORIGIN_LON,
    speed_tolerance_mps: float = DEFAULT_SPEED_TOLERANCE_MPS,
) -> Tuple[Dict, Dict]:
    """Build a single-speed straight-line mission + meta.

    Layout per trial t (alternating direction):
        hold(1 s)              at p0_t
        warmup_end_t           at p0_t + 30 m * dir
        meas_end_t             at warmup_end_t + 50 m * dir   <- snapshot end
        cooldown_end_t         at meas_end_t + 30 m * dir

    The next trial's hold is at cooldown_end_t (which is p0_{t+1} for the
    reverse direction), so the corridor is exactly 110 m long and shared
    across all trials.

    Snapshot spans (warmup_end_t, meas_end_t). The next item after meas_end_t
    is cooldown_end_t (a navigation/waypoint, smooth pass-through), so the
    drone is at v during the entire snapshot — deceleration only kicks in
    once cooldown_end_t is followed by a `hold`.
    """
    items: List[Dict] = [{"type": "takeoff"}]
    items.append({
        "type": "changeSettings",
        "horizontalVelocity": float(speed_mps),
        "verticalVelocity": 0.5,
        "maxHeadingRate": float(max_heading_rate_deg_s),
    })

    leg_total_m = warmup_m + measure_m + cooldown_m
    legs = []
    for t in range(n_trials):
        # Direction along x: trial 0 east (+1), trial 1 west (-1), ...
        direction = 1 if (t % 2 == 0) else -1
        if direction == 1:
            x0 = 0.0
        else:
            x0 = leg_total_m

        # hold(1 s) at the trial's starting point
        items.append({"type": "hold", "duration": 1.0})

        warmup_end_x = x0 + direction * warmup_m
        meas_end_x = warmup_end_x + direction * measure_m
        cooldown_end_x = meas_end_x + direction * cooldown_m

        warmup_end_idx = len(items)
        items.append(_waypoint_global(
            f"sl_warmup_end_{t}", warmup_end_x, 0.0, altitude_m,
            origin_lat, origin_lon,
        ))
        items.append(_waypoint_global(
            f"sl_meas_end_{t}", meas_end_x, 0.0, altitude_m,
            origin_lat, origin_lon,
        ))
        items.append(_waypoint_global(
            f"sl_cooldown_end_{t}", cooldown_end_x, 0.0, altitude_m,
            origin_lat, origin_lon,
        ))

        legs.append({
            "start_idx": warmup_end_idx,
            "end_idx": warmup_end_idx + 1,  # meas_end is the next item
            "trial_id": f"v{int(round(speed_mps)):02d}_t{t:02d}",
            "condition": {
                "experiment": "straight_line",
                "speed_mps": float(speed_mps),
                "direction": "east" if direction == 1 else "west",
                "trial_index": t,
            },
            "expected_distance_m": measure_m,
            "expected_speed_mps": float(speed_mps),
        })

    items.append({"type": "rtl"})

    mission = {
        "version": 1,
        "mission": {
            "defaults": {
                "horizontalVelocity": float(speed_mps),
                "verticalVelocity": 0.5,
                "maxHeadingRate": float(max_heading_rate_deg_s),
            },
            "items": items,
        },
    }
    meta = {
        "experiment": "straight_line",
        "csv_path": csv_path,
        "speed_tolerance_mps": speed_tolerance_mps,
        "legs": legs,
    }
    return mission, meta


# ---------------------------------------------------------------------------
# Turn missions
# ---------------------------------------------------------------------------

def gen_turn_mission(
    v_g_mps: float,
    omega_rad_s: float,
    n_trials: int,
    turn_angles_rad: Tuple[float, ...],
    csv_path: str,
    altitude_m: float = DEFAULT_ALTITUDE_M,
    warmup_m: float = DEFAULT_WARMUP_M,
    leg_m: float = DEFAULT_TURN_LEG_M,
    cooldown_m: float = DEFAULT_COOLDOWN_M,
    cell_x_m: float = 250.0,
    cell_y_m: float = 250.0,
    mpc_acc_hor_mps2: float = DEFAULT_MPC_ACC_HOR_MPS2,
    origin_lat: float = DEFAULT_ORIGIN_LAT,
    origin_lon: float = DEFAULT_ORIGIN_LON,
    speed_tolerance_mps: float = DEFAULT_SPEED_TOLERANCE_MPS,
) -> Tuple[Dict, Dict]:
    """Build a single-(v_g, omega) turn mission with all 5 angles × n_trials.

    Trials are laid out on a (n_trials × len(angles)) grid: column = angle,
    row = trial. Each cell is `cell_x_m × cell_y_m`. With cell_x_m = 250 m
    and 5 angles × 7 trials = 35 trials, the total area is 1250 m × 1750 m —
    well within the SITL geofence.

    Per trial, the per-leg metadata `start_idx` is at i (smooth pass-through
    at v_g), `end_idx` at cooldown_end (smooth pass-through at v_g). The
    drone is at v_g at both boundaries because:
        * warmup → i: drone accelerates over `warmup_m` and reaches v_g by i.
        * j → cooldown_end: drone exits hold at j, accelerates 0 → v_g over
          `leg_m + cooldown_m` (40 m at default cooldown), reaches v_g
          before cooldown_end, holds at v_g (next item is hold → decel
          starts AFTER snapshot end).

    Expected energy reduction (used by the fit):
        E_turn_observed = E_total - P_translate(v_g) * (warmup + 2*leg + cooldown) / v_g
                        = E_total - P_translate(v_g) * 100 / v_g     (defaults)

    The intercept E_acc absorbs any constant offset in the model.
    """
    items: List[Dict] = [{"type": "takeoff"}]
    items.append({
        "type": "changeSettings",
        "horizontalVelocity": float(v_g_mps),
        "verticalVelocity": 0.5,
        "maxHeadingRate": math.degrees(omega_rad_s),
    })

    legs = []
    for t in range(n_trials):
        for a, delta_theta in enumerate(turn_angles_rad):
            x0 = a * cell_x_m
            y0 = t * cell_y_m

            # Adaptive hold so decel + yaw fit before the next item activates.
            hold_s = _adaptive_turn_hold_s(
                v_g_mps, omega_rad_s, delta_theta, mpc_acc_hor_mps2,
            )

            # Warmup_start: drone is at rest after the preceding hold.
            items.append({"type": "hold", "duration": 1.0})

            warmup_end_x = x0 + warmup_m
            warmup_end_y = y0
            i_x, i_y = warmup_end_x, warmup_end_y       # i coincides with warmup_end
            j_x, j_y = i_x + leg_m, i_y                 # j is leg_m east of i
            k_x = j_x + leg_m * math.cos(delta_theta)   # rotate by Δθ from +x
            k_y = j_y + leg_m * math.sin(delta_theta)
            cool_x = j_x + (leg_m + cooldown_m) * math.cos(delta_theta)
            cool_y = j_y + (leg_m + cooldown_m) * math.sin(delta_theta)

            warmup_idx = len(items)
            items.append(_waypoint_global(
                f"tr_warmup_{t}_{a}", warmup_end_x, warmup_end_y, altitude_m,
                origin_lat, origin_lon,
            ))
            i_idx = len(items)
            items.append(_waypoint_global(
                f"tr_i_{t}_{a}", i_x, i_y, altitude_m, origin_lat, origin_lon,
            ))
            items.append(_waypoint_global(
                f"tr_j_{t}_{a}", j_x, j_y, altitude_m, origin_lat, origin_lon,
            ))
            items.append({"type": "hold", "duration": float(hold_s)})
            items.append(_waypoint_global(
                f"tr_k_{t}_{a}", k_x, k_y, altitude_m, origin_lat, origin_lon,
            ))
            cool_idx = len(items)
            items.append(_waypoint_global(
                f"tr_cooldown_{t}_{a}", cool_x, cool_y, altitude_m,
                origin_lat, origin_lon,
            ))

            # The straight-line distance the model predicts the drone covers
            # at v_g during the snapshot is warmup→i (0, warmup_end and i
            # coincide) + i→j (leg) + j→k (leg) + k→cooldown (cooldown_m).
            # Total: leg + leg + cooldown = 2*leg + cooldown.
            # Note: warmup is BEFORE i, and the snapshot starts at i, so the
            # warmup distance is NOT in the snapshot.
            expected_d = 2 * leg_m + cooldown_m
            expected_v = float(v_g_mps)

            legs.append({
                "start_idx": i_idx,        # snapshot starts at i (smooth pass at v_g)
                "end_idx": cool_idx,       # snapshot ends at cooldown_end (smooth pass at v_g)
                "trial_id": f"vg{int(round(v_g_mps)):02d}_w{omega_rad_s:.1f}_a{a}_t{t:02d}",
                "condition": {
                    "experiment": "turn",
                    "v_g_mps": float(v_g_mps),
                    "omega_rad_s": float(omega_rad_s),
                    "delta_theta_rad": float(delta_theta),
                    "trial_index": t,
                    "angle_index": a,
                    "predicted_straight_line_m": expected_d,
                    "warmup_m": warmup_m,
                    "leg_m": leg_m,
                    "cooldown_m": cooldown_m,
                    "hold_duration_s": hold_s,
                },
                "expected_distance_m": expected_d,
                "expected_speed_mps": expected_v,
            })

    items.append({"type": "rtl"})

    omega_str = f"{omega_rad_s:.1f}".replace(".", "p")
    mission = {
        "version": 1,
        "mission": {
            "defaults": {
                "horizontalVelocity": float(v_g_mps),
                "verticalVelocity": 0.5,
                "maxHeadingRate": math.degrees(omega_rad_s),
            },
            "items": items,
        },
    }
    meta = {
        "experiment": "turn",
        "csv_path": csv_path,
        "speed_tolerance_mps": speed_tolerance_mps,
        "legs": legs,
    }
    return mission, meta


# ---------------------------------------------------------------------------
# Top-level: emit the entire suite
# ---------------------------------------------------------------------------

def gen_all(
    output_dir: str,
    n_trials: int = DEFAULT_N_TRIALS,
    speeds: Tuple[float, ...] = DEFAULT_SPEEDS_MPS,
    turn_v_g: Tuple[float, ...] = DEFAULT_TURN_VG_MPS,
    turn_omega: Tuple[float, ...] = DEFAULT_TURN_OMEGA,
    turn_angles: Tuple[float, ...] = DEFAULT_TURN_ANGLES,
    altitude_m: float = DEFAULT_ALTITUDE_M,
    hover_duration_s: float = DEFAULT_HOVER_DURATION_S,
    speed_tolerance_mps: float = DEFAULT_SPEED_TOLERANCE_MPS,
    origin_lat: float = DEFAULT_ORIGIN_LAT,
    origin_lon: float = DEFAULT_ORIGIN_LON,
    csv_dir: str = None,
) -> List[Dict]:
    """Emit every mission in the suite under output_dir/.

    Returns a list of dicts describing each mission:
        {"name", "mission_path", "meta_path", "experiment"}

    `csv_dir` defaults to output_dir/calibration_data/.
    """
    out = Path(output_dir)
    (out / "missions").mkdir(parents=True, exist_ok=True)
    if csv_dir is None:
        csv_dir = out / "calibration_data"
    csv_dir = Path(csv_dir)
    csv_dir.mkdir(parents=True, exist_ok=True)

    manifest: List[Dict] = []

    def _write_pair(name: str, mission: Dict, meta: Dict, experiment: str):
        mission_path = out / "missions" / f"{name}.mission.json"
        meta_path = out / "missions" / f"{name}.meta.json"
        with mission_path.open("w") as f:
            json.dump(mission, f, indent=2)
        with meta_path.open("w") as f:
            json.dump(meta, f, indent=2)
        manifest.append({
            "name": name,
            "mission_path": str(mission_path),
            "meta_path": str(meta_path),
            "experiment": experiment,
        })

    # 1. Hover
    hover_csv = str(csv_dir / "hover.csv")
    mission, meta = gen_hover_mission(
        n_trials=n_trials,
        csv_path=hover_csv,
        altitude_m=altitude_m,
        hover_duration_s=hover_duration_s,
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        speed_tolerance_mps=speed_tolerance_mps,
    )
    _write_pair("hover", mission, meta, "hover")

    # 2. Straight-line (one mission per speed)
    straight_csv = str(csv_dir / "straight_line.csv")
    for v in speeds:
        mission, meta = gen_straight_line_mission(
            speed_mps=v,
            n_trials=n_trials,
            csv_path=straight_csv,
            altitude_m=altitude_m,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            speed_tolerance_mps=speed_tolerance_mps,
        )
        _write_pair(
            f"straight_line_v{int(round(v)):02d}", mission, meta, "straight_line",
        )

    # 3. Turn (one mission per (v_g, omega) combination)
    turn_csv = str(csv_dir / "turns.csv")
    for v_g in turn_v_g:
        for omega in turn_omega:
            mission, meta = gen_turn_mission(
                v_g_mps=v_g,
                omega_rad_s=omega,
                n_trials=n_trials,
                turn_angles_rad=turn_angles,
                csv_path=turn_csv,
                altitude_m=altitude_m,
                origin_lat=origin_lat,
                origin_lon=origin_lon,
                speed_tolerance_mps=speed_tolerance_mps,
            )
            omega_str = f"{omega:.1f}".replace(".", "p")
            _write_pair(
                f"turn_vg{int(round(v_g)):02d}_omega{omega_str}",
                mission, meta, "turn",
            )

    # Manifest
    manifest_path = out / "missions" / "calibration_manifest.json"
    with manifest_path.open("w") as f:
        json.dump({
            "n_trials": n_trials,
            "speeds": list(speeds),
            "turn_v_g": list(turn_v_g),
            "turn_omega": list(turn_omega),
            "turn_angles": list(turn_angles),
            "missions": manifest,
        }, f, indent=2)
    return manifest


def main():
    parser = argparse.ArgumentParser(
        description="Generate energy-cost calibration missions",
    )
    parser.add_argument(
        "--output-dir", required=True,
        help="Directory to write missions/, meta/, calibration_data/ under",
    )
    parser.add_argument(
        "--n-trials", type=int, default=DEFAULT_N_TRIALS,
        help=f"Trials per condition (default {DEFAULT_N_TRIALS})",
    )
    parser.add_argument(
        "--altitude", type=float, default=DEFAULT_ALTITUDE_M,
        help=f"Patrol altitude in meters (default {DEFAULT_ALTITUDE_M})",
    )
    parser.add_argument(
        "--hover-duration", type=float, default=DEFAULT_HOVER_DURATION_S,
        help=f"Hover trial duration in seconds (default {DEFAULT_HOVER_DURATION_S})",
    )
    parser.add_argument(
        "--domain-config", default=DEFAULT_DOMAIN_CONFIG,
        help=(
            "shepherd-rl domain config JSON. The named starting_point zone "
            "becomes the local Cartesian origin for all missions "
            f"(default {DEFAULT_DOMAIN_CONFIG})"
        ),
    )
    parser.add_argument(
        "--starting-point-name", default=DEFAULT_STARTING_POINT_NAME,
        help=(
            "Which starting_point zone to use when the config has multiple "
            f"(default {DEFAULT_STARTING_POINT_NAME!r})"
        ),
    )
    parser.add_argument(
        "--origin-lat", type=float, default=None,
        help=(
            "Override the latitude extracted from --domain-config. Pair with "
            "--origin-lon. Both must be provided together."
        ),
    )
    parser.add_argument(
        "--origin-lon", type=float, default=None,
        help="Override the longitude extracted from --domain-config.",
    )
    args = parser.parse_args()

    if (args.origin_lat is None) != (args.origin_lon is None):
        parser.error(
            "--origin-lat and --origin-lon must be provided together"
        )
    if args.origin_lat is None:
        origin_lat, origin_lon = extract_starting_point(
            args.domain_config, args.starting_point_name,
        )
    else:
        origin_lat, origin_lon = args.origin_lat, args.origin_lon

    manifest = gen_all(
        output_dir=args.output_dir,
        n_trials=args.n_trials,
        altitude_m=args.altitude,
        hover_duration_s=args.hover_duration,
        origin_lat=origin_lat,
        origin_lon=origin_lon,
    )
    print(f"Generated {len(manifest)} missions under {args.output_dir}/missions/")
    for entry in manifest:
        print(f"  - {entry['name']:<30s} ({entry['experiment']})")


if __name__ == "__main__":
    main()
