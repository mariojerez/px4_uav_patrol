"""Tests for gen_calibration_missions.py: validates that the generated
mission and meta JSONs are well-formed, snapshot indices line up with the
mission item indices they reference, and the per-leg geometry matches the
plan."""

import json
import math
import sys
import tempfile
from pathlib import Path

import pytest

# The package is installed under share/px4_uav_patrol/calibration/ via colcon
# install. The module under test is a plain Python file there. Easiest
# approach: import it directly by absolute path.
HERE = Path(__file__).resolve().parent
CALIBRATION_DIR = HERE.parent / "calibration"
sys.path.insert(0, str(CALIBRATION_DIR))

import gen_calibration_missions as gcm  # noqa: E402


# -----------------------------------------------------------------------------
# Hover mission
# -----------------------------------------------------------------------------

def test_hover_mission_structure():
    mission, meta = gcm.gen_hover_mission(
        n_trials=7, csv_path="/tmp/hover.csv",
    )
    items = mission["mission"]["items"]
    # takeoff + changeSettings + 7 * (waypoint, hold) + terminal waypoint + rtl
    assert len(items) == 1 + 1 + 7 * 2 + 1 + 1
    assert items[0]["type"] == "takeoff"
    assert items[1]["type"] == "changeSettings"
    assert items[1]["maxHeadingRate"] == 0.0
    assert items[-1]["type"] == "rtl"

    # 7 hover holds at indices 3, 5, 7, ... 15
    hold_indices = [i for i, item in enumerate(items) if item.get("type") == "hold"]
    assert hold_indices == [3, 5, 7, 9, 11, 13, 15]
    for idx in hold_indices:
        assert items[idx]["duration"] == pytest.approx(25.0)

    # 7 legs in meta, each spanning anchor→next anchor.
    assert len(meta["legs"]) == 7
    for t, leg in enumerate(meta["legs"]):
        expected_start = 2 + 2 * t
        expected_end = 2 + 2 * (t + 1)
        assert leg["start_idx"] == expected_start
        assert leg["end_idx"] == expected_end
        assert items[expected_start].get("navigationType") == "waypoint"
        # end_idx points at either the next anchor or the terminal anchor
        assert items[expected_end].get("navigationType") == "waypoint"
        assert leg["expected_speed_mps"] == 0.0


def test_hover_mission_n_trials_override():
    _, meta = gcm.gen_hover_mission(n_trials=3, csv_path="/tmp/h.csv")
    assert len(meta["legs"]) == 3


# -----------------------------------------------------------------------------
# Straight-line missions
# -----------------------------------------------------------------------------

def test_straight_line_mission_structure():
    mission, meta = gcm.gen_straight_line_mission(
        speed_mps=4.0, n_trials=7, csv_path="/tmp/sl.csv",
    )
    items = mission["mission"]["items"]
    # takeoff + changeSettings + 7 * (hold + warmup_end + meas_end + cooldown_end) + rtl
    assert len(items) == 1 + 1 + 7 * 4 + 1
    assert items[0]["type"] == "takeoff"
    assert items[-1]["type"] == "rtl"
    assert items[1]["horizontalVelocity"] == pytest.approx(4.0)

    assert len(meta["legs"]) == 7
    for t, leg in enumerate(meta["legs"]):
        # hold at 2 + 4*t, warmup_end at 2 + 4*t + 1, meas_end at 2 + 4*t + 2
        warmup_idx = 2 + 4 * t + 1
        meas_idx = 2 + 4 * t + 2
        assert leg["start_idx"] == warmup_idx
        assert leg["end_idx"] == meas_idx
        # start and end indices must be navigation/waypoint items
        assert items[warmup_idx].get("navigationType") == "waypoint"
        assert items[meas_idx].get("navigationType") == "waypoint"
        # The item right BEFORE the warmup must be a hold (drone at rest).
        assert items[warmup_idx - 1]["type"] == "hold"
        # The item right AFTER meas_end must be a navigation/waypoint
        # (cooldown_end), NOT a hold — otherwise the drone would decelerate
        # AT meas_end and contaminate the snapshot.
        assert items[meas_idx + 1].get("navigationType") == "waypoint"
        assert leg["expected_distance_m"] == pytest.approx(50.0)
        assert leg["expected_speed_mps"] == pytest.approx(4.0)


def test_straight_line_alternates_direction():
    # Convert global lon/lat back to relative meters by inverting local_to_global.
    mission, meta = gcm.gen_straight_line_mission(
        speed_mps=4.0, n_trials=4, csv_path="/tmp/sl.csv",
        origin_lat=0.0, origin_lon=0.0,  # use 0 so x=lon*METERS_PER_DEG
    )
    items = mission["mission"]["items"]
    # warmup_end of trial 0 is east of origin; trial 1 is west of trial 0's
    # cooldown_end (= 110m east). So warmup_end_1 (longitude -> x_m) should be
    # less than warmup_end_0 (i.e. heading west).
    warmup0 = items[meta["legs"][0]["start_idx"]]
    warmup1 = items[meta["legs"][1]["start_idx"]]
    # x = longitude. With origin_lon=0, x_m = lon * METERS_PER_DEG_LAT * cos(0)
    # → larger lon means more east.
    assert warmup0["x"] < warmup1["x"], (
        "Trial 0 should be at warmup_end (30 m east of origin) and trial 1 "
        "should be at warmup_end_1 (80 m east of origin, heading west)"
    )


# -----------------------------------------------------------------------------
# Turn missions
# -----------------------------------------------------------------------------

def test_turn_mission_structure():
    angles = (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4, math.pi)
    mission, meta = gcm.gen_turn_mission(
        v_g_mps=4.0, omega_rad_s=1.0, n_trials=7,
        turn_angles_rad=angles, csv_path="/tmp/turn.csv",
    )
    items = mission["mission"]["items"]
    # takeoff + changeSettings + 7 * 5 * (hold + warmup + i + j + hold + k + cooldown) + rtl
    # = 2 + 35 * 7 + 1 = 248
    expected_items = 2 + len(angles) * 7 * 7 + 1
    assert len(items) == expected_items
    assert items[0]["type"] == "takeoff"
    assert items[-1]["type"] == "rtl"

    # 35 legs total
    assert len(meta["legs"]) == 7 * len(angles)
    for leg in meta["legs"]:
        # start_idx → i (waypoint), end_idx → cooldown_end (waypoint)
        assert items[leg["start_idx"]].get("navigationType") == "waypoint"
        assert items[leg["end_idx"]].get("navigationType") == "waypoint"

        # Between i and cooldown_end: j waypoint, hold, k waypoint
        between = items[leg["start_idx"] + 1: leg["end_idx"]]
        assert between[0].get("navigationType") == "waypoint"  # j
        assert between[1]["type"] == "hold"
        assert between[2].get("navigationType") == "waypoint"  # k
        # The hold duration is adaptive: at omega=1.0 rad/s, Δθ=π → ≥ pi seconds + decel + margin.
        cond = leg["condition"]
        delta = cond["delta_theta_rad"]
        assert between[1]["duration"] >= 2.0  # floor
        # roughly pi/omega + v_g/acc + 1
        expected_min = (delta / 1.0) + (4.0 / gcm.DEFAULT_MPC_ACC_HOR_MPS2) + 1.0
        assert between[1]["duration"] >= max(2.0, expected_min) - 1e-6


def test_turn_mission_geometry_for_pi_turn():
    angles = (math.pi,)
    mission, meta = gcm.gen_turn_mission(
        v_g_mps=4.0, omega_rad_s=1.0, n_trials=1,
        turn_angles_rad=angles, csv_path="/tmp/turn.csv",
        origin_lat=0.0, origin_lon=0.0,
        cell_x_m=0.0, cell_y_m=0.0,  # single trial at origin
    )
    items = mission["mission"]["items"]
    # The leg's k waypoint should be 20 m back toward origin from j (since
    # delta_theta = π, drone reverses direction).
    leg = meta["legs"][0]
    i = items[leg["start_idx"]]
    j = items[leg["start_idx"] + 1]
    k = items[leg["start_idx"] + 3]  # +1=j, +2=hold, +3=k

    def lon_to_x_m(lon):
        return lon * gcm.METERS_PER_DEG_LAT * math.cos(math.radians(0.0))

    # i is at warmup_end (30 m east of warmup_start=origin)
    assert lon_to_x_m(i["x"]) == pytest.approx(30.0, abs=0.5)
    assert lon_to_x_m(j["x"]) == pytest.approx(50.0, abs=0.5)
    # k is 20 m WEST of j (back toward i) for delta_theta = π
    assert lon_to_x_m(k["x"]) == pytest.approx(30.0, abs=0.5)


# -----------------------------------------------------------------------------
# gen_all integration
# -----------------------------------------------------------------------------

def test_gen_all_writes_full_suite():
    with tempfile.TemporaryDirectory() as tmp:
        manifest = gcm.gen_all(output_dir=tmp, n_trials=2)

        # 1 hover + 5 straight-line + 9 turn = 15 missions
        assert len(manifest) == 15
        names = {entry["name"] for entry in manifest}
        assert "hover" in names
        for v in (2, 4, 6, 8, 10):
            assert f"straight_line_v{v:02d}" in names
        for vg in (2, 4, 6):
            for omega in ("0p5", "1p0", "1p5"):
                assert f"turn_vg{vg:02d}_omega{omega}" in names

        # Each entry has a real on-disk pair.
        for entry in manifest:
            assert Path(entry["mission_path"]).exists()
            assert Path(entry["meta_path"]).exists()
            with open(entry["mission_path"]) as f:
                mission = json.load(f)
            with open(entry["meta_path"]) as f:
                meta = json.load(f)
            # Every leg's start/end indices are inside the items array.
            n_items = len(mission["mission"]["items"])
            for leg in meta["legs"]:
                assert 0 <= leg["start_idx"] < n_items
                assert 0 <= leg["end_idx"] < n_items
                assert leg["end_idx"] > leg["start_idx"]


def test_gen_all_n_trials_override_propagates():
    with tempfile.TemporaryDirectory() as tmp:
        manifest = gcm.gen_all(output_dir=tmp, n_trials=3)
        for entry in manifest:
            with open(entry["meta_path"]) as f:
                meta = json.load(f)
            if entry["experiment"] == "hover":
                assert len(meta["legs"]) == 3
            elif entry["experiment"] == "straight_line":
                assert len(meta["legs"]) == 3
            elif entry["experiment"] == "turn":
                # 3 trials × 5 angles = 15 legs
                assert len(meta["legs"]) == 15
