"""Synthetic-data tests for fit_energy_constants.py.

Generates trial CSVs from known constants, runs the fit, and asserts the
recovered constants match within a small tolerance. The synthetic data
mimics the per-leg CSV the real EnergyLogger emits so the tests exercise
the same code paths as production runs.
"""

import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest
import yaml

HERE = Path(__file__).resolve().parent
CALIBRATION_DIR = HERE.parent / "calibration"
sys.path.insert(0, str(CALIBRATION_DIR))

import fit_energy_constants as fec  # noqa: E402


# Ground truth: realistic constants for a small quadcopter.
GT_LAMBDA_1 = 0.05         # W / (m/s)²
GT_LAMBDA_2 = 0.30         # W / (m/s)
GT_P_HOVER = 180.0         # W
GT_E_ACC = 200.0           # J
GT_GAMMA = 50.0            # J / rad


def _p_translate(v):
    return GT_LAMBDA_1 * v * v + GT_LAMBDA_2 * v + GT_P_HOVER


CSV_HEADER = (
    "experiment,trial_id,condition_json,start_idx,end_idx,"
    "energy_j,distance_m,duration_s,"
    "mean_power_w,mean_speed_mps,mean_voltage_v,"
    "expected_distance_m,expected_speed_mps,"
    "passed_speed_check,utc_s\n"
)


def _csv_quote(s: str) -> str:
    return '"' + s.replace('"', '""') + '"'


def _write_synthetic_csvs(
    tmp: Path, n_trials: int = 7, noise_std_w: float = 1.0,
    rng: np.random.Generator = None,
):
    """Create hover.csv, straight_line.csv, turns.csv with known ground truth."""
    if rng is None:
        rng = np.random.default_rng(42)
    tmp.mkdir(parents=True, exist_ok=True)

    # Hover
    with (tmp / "hover.csv").open("w") as f:
        f.write(CSV_HEADER)
        for t in range(n_trials):
            duration = 25.0
            mean_power_w = float(GT_P_HOVER + rng.normal(0, noise_std_w))
            energy_j = mean_power_w * duration
            cond = {"experiment": "hover", "trial_index": t,
                    "speed_mps": 0.0}
            f.write(
                f"hover,h{t:02d},{_csv_quote(json.dumps(cond))},"
                f"2,3,{energy_j},0.0,{duration},{mean_power_w},0.0,16.0,"
                f"0.0,0.0,true,1.0\n"
            )

    # Straight line
    speeds = (2.0, 4.0, 6.0, 8.0, 10.0)
    distance_m = 50.0
    with (tmp / "straight_line.csv").open("w") as f:
        f.write(CSV_HEADER)
        for v in speeds:
            for t in range(n_trials):
                duration = distance_m / v
                p_true = _p_translate(v)
                mean_power_w = float(p_true + rng.normal(0, noise_std_w))
                energy_j = mean_power_w * duration
                cond = {"experiment": "straight_line",
                        "speed_mps": v, "trial_index": t}
                f.write(
                    f"straight_line,v{int(v):02d}_t{t:02d},"
                    f"{_csv_quote(json.dumps(cond))},"
                    f"3,4,{energy_j},{distance_m},{duration},"
                    f"{mean_power_w},{v},16.0,"
                    f"{distance_m},{v},true,1.0\n"
                )

    # Turn — model: E_total = P_translate(v_g) * d_predicted/v_g + E_acc + gamma*Δθ
    angles = (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4, math.pi)
    v_gs = (2.0, 4.0, 6.0)
    omegas = (0.5, 1.0, 1.5)
    d_pred = 70.0   # 2*leg + cooldown = 2*20 + 30
    expected_v = None  # set below per row
    with (tmp / "turns.csv").open("w") as f:
        f.write(CSV_HEADER)
        for v_g in v_gs:
            for omega in omegas:
                for t in range(n_trials):
                    for a, delta in enumerate(angles):
                        e_straight = _p_translate(v_g) * d_pred / v_g
                        e_turn_true = GT_E_ACC + GT_GAMMA * delta
                        e_total = float(
                            e_straight + e_turn_true
                            + rng.normal(0, noise_std_w * 5)
                        )
                        # Distance ≈ d_pred + small jitter
                        dist = d_pred
                        duration = dist / v_g
                        mean_power_w = e_total / duration
                        cond = {
                            "experiment": "turn",
                            "v_g_mps": v_g,
                            "omega_rad_s": omega,
                            "delta_theta_rad": delta,
                            "trial_index": t,
                            "angle_index": a,
                            "predicted_straight_line_m": d_pred,
                        }
                        f.write(
                            f"turn,vg{int(v_g):02d}_w{omega:.1f}_a{a}_t{t:02d},"
                            f"{_csv_quote(json.dumps(cond))},"
                            f"3,8,{e_total},{dist},{duration},"
                            f"{mean_power_w},{v_g},16.0,"
                            f"{dist},{v_g},true,1.0\n"
                        )


def test_fit_recovers_known_constants():
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        data = tmp / "data"
        out = tmp / "out"
        _write_synthetic_csvs(data)
        fec.fit_all(str(data), str(out))

        with (out / "energy_constants.yaml").open() as f:
            constants = yaml.safe_load(f)
        sl = constants["straight_line"]
        tu = constants["turn"]

        # Individual lambda_1 and lambda_2 are highly anti-correlated in
        # this fit (pcov is structurally near-singular for a quadratic
        # over a small speed range), so the fit can swap a quadratic
        # contribution for a linear one without changing the prediction.
        # Validate the *prediction* at each test speed instead — that's
        # what consumers of energy_constants.yaml care about.
        for v in (0.0, 2.0, 4.0, 6.0, 8.0, 10.0):
            true_p = _p_translate(v)
            fit_p = (sl["lambda_1"] * v * v
                     + sl["lambda_2"] * v + sl["P_hover"])
            tol_w = max(2.0, 0.02 * true_p)  # 2 W or 2 % of true, whichever larger
            assert abs(fit_p - true_p) < tol_w, (
                f"P({v}) fit {fit_p:.2f} differs from {true_p:.2f} by > {tol_w:.2f} W"
            )
        assert abs(sl["P_hover"] - GT_P_HOVER) / GT_P_HOVER < 0.05, (
            f"P_hover fit {sl['P_hover']} far from {GT_P_HOVER}"
        )
        assert abs(tu["E_acc"] - GT_E_ACC) / GT_E_ACC < 0.05, (
            f"E_acc fit {tu['E_acc']} far from {GT_E_ACC}"
        )
        assert abs(tu["gamma"] - GT_GAMMA) / GT_GAMMA < 0.05, (
            f"gamma fit {tu['gamma']} far from {GT_GAMMA}"
        )

        # R² depends on noise vs true variation. With 1 W per-trial noise
        # and only ~8 W of P_translate variation across the 0-10 m/s range,
        # the inter-speed signal is comparable to noise — R² lands ~0.9 even
        # for a perfect model. The turn fit has a much larger signal range
        # (γ·π ≈ 157 J vs noise ~5 J), so R² is much higher there.
        assert sl["R2"] > 0.85
        assert all(g["R2"] > 0.95 for g in tu["per_group"])


def test_outputs_present():
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        data = tmp / "data"
        out = tmp / "out"
        _write_synthetic_csvs(data)
        fec.fit_all(str(data), str(out))
        assert (out / "energy_constants.yaml").exists()
        assert (out / "plots" / "p_translate.png").exists()
        assert (out / "plots" / "e_turn.png").exists()


def test_speed_check_filtering():
    """Trials with passed_speed_check=false must be excluded from the fit."""
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        data = tmp / "data"
        out = tmp / "out"
        _write_synthetic_csvs(data)
        # Append a corrupted row to straight_line.csv
        with (data / "straight_line.csv").open("a") as f:
            cond = {"experiment": "straight_line", "speed_mps": 4.0,
                    "trial_index": 99}
            f.write(
                f"straight_line,corrupt,{_csv_quote(json.dumps(cond))},"
                "3,4,99999.0,50.0,12.5,7999.9,4.0,16.0,50.0,4.0,false,1.0\n"
            )
        fec.fit_all(str(data), str(out))
        with (out / "energy_constants.yaml").open() as f:
            c = yaml.safe_load(f)
        assert c["fit_metadata"]["trials_excluded_by_speed_check"] >= 1
        # Fit should still recover constants closely (corrupted row was
        # excluded).
        assert abs(c["straight_line"]["P_hover"] - GT_P_HOVER) / GT_P_HOVER < 0.05


def test_fit_idempotent():
    """Running fit_all twice on the same data produces identical numbers."""
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        data = tmp / "data"
        out1 = tmp / "out1"
        out2 = tmp / "out2"
        _write_synthetic_csvs(data, rng=np.random.default_rng(7))
        c1 = fec.fit_all(str(data), str(out1))
        c2 = fec.fit_all(str(data), str(out2))
        for k in ("lambda_1", "lambda_2", "P_hover"):
            assert c1["straight_line"][k] == c2["straight_line"][k]
        for k in ("E_acc", "gamma"):
            assert c1["turn"][k] == c2["turn"][k]
