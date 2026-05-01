"""Fit the four energy-cost constants from the per-trial CSVs and write
`energy_constants.yaml` + diagnostic plots.

Inputs (under `--data-dir`):
  hover.csv            (n_trials rows, v_g = 0 m/s)
  straight_line.csv    (5 speeds × n_trials rows)
  turns.csv            (9 (v_g,ω) × 5 angles × n_trials rows)

Outputs (under `--output-dir`):
  energy_constants.yaml
  plots/p_translate.png
  plots/e_turn.png

Fit model:
  Straight-line:  P_translate(v) = lambda_1 * v^2 + lambda_2 * v + P_hover
                  fitted with scipy.optimize.curve_fit on per-trial mean
                  power. The v=0 point comes from hover.csv.
  Turn:           For each trial, E_turn = E_total - P_translate(v_g) *
                  predicted_straight_line_m / v_g. Then per (v_g, ω) group,
                  linear-regress E_turn against delta_theta — slope = gamma,
                  intercept = E_acc. Final reported gamma and E_acc are
                  the mean ± std across the 9 groups.

Trials with `passed_speed_check == False` are kept in the CSV but excluded
from the fit.
"""

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import scipy.optimize
import scipy.stats
import yaml


def _read_csv_rows(path: Path) -> List[Dict]:
    """Read the per-trial CSV emitted by EnergyLogger.

    Format (one header line):
      experiment,trial_id,condition_json,start_idx,end_idx,
      energy_j,distance_m,duration_s,
      mean_power_w,mean_speed_mps,mean_voltage_v,
      expected_distance_m,expected_speed_mps,
      passed_speed_check,utc_s

    `condition_json` is a quoted JSON blob that may contain commas; we strip
    it out manually to keep the CSV reader simple.
    """
    rows: List[Dict] = []
    if not path.exists():
        return rows
    with path.open() as f:
        header = f.readline().strip().split(",")
        for line in f:
            line = line.rstrip("\n")
            # Find the quoted condition_json field
            assert line.count('"') >= 2, f"Malformed CSV line: {line}"
            first_q = line.find('"')
            second_q = line.find('"', first_q + 1)
            # Remaining quotes inside the field must be CSV-escaped as ""
            # — keep advancing second_q while it is followed by another ".
            while second_q + 1 < len(line) and line[second_q + 1] == '"':
                second_q = line.find('"', second_q + 2)
            condition_json = line[first_q + 1:second_q].replace('""', '"')
            before = line[:first_q].rstrip(",").split(",")
            after = line[second_q + 1:].lstrip(",").split(",")
            fields = before + [condition_json] + after
            if len(fields) != len(header):
                # Skip malformed rows rather than crash the fit.
                continue
            row = dict(zip(header, fields))
            row["condition"] = json.loads(condition_json)
            rows.append(row)
    return rows


def _filter_passed(rows: List[Dict]) -> Tuple[List[Dict], int]:
    excluded = sum(1 for r in rows if r["passed_speed_check"].lower() != "true")
    return [r for r in rows if r["passed_speed_check"].lower() == "true"], excluded


def _power_law(v, lambda_1, lambda_2, p_hover):
    return lambda_1 * v * v + lambda_2 * v + p_hover


def fit_straight_line(
    straight_rows: List[Dict], hover_rows: List[Dict],
) -> Dict:
    """Fit P_translate(v) = lambda_1 v² + lambda_2 v + P_hover.

    Each trial contributes one point: (v_measured, mean_power_w). Hover
    trials are treated as v=0 points using their mean_power_w. Using the
    measured speed (not the commanded one) means PX4 SITL trials that fell
    short of their commanded v_g still contribute a valid (v, P) sample at
    their actual cruise speed — without this, the v ≥ 6 m/s rows that hit
    the SITL speed cap would have to be tossed, leaving too few speed
    samples to fit curvature.
    """
    # Each entry: (v_measured, P, V, commanded_v) — commanded_v keeps the
    # per-speed diagnostic grouping intuitive even though the fit uses
    # measured speeds.
    points: List[Tuple[float, float, float, float]] = []
    for row in straight_rows:
        v_meas = float(row["mean_speed_mps"])
        v_cmd = float(row["condition"]["speed_mps"])
        p = float(row["mean_power_w"])
        V = float(row["mean_voltage_v"])
        points.append((v_meas, p, V, v_cmd))
    for row in hover_rows:
        p = float(row["mean_power_w"])
        V = float(row["mean_voltage_v"])
        points.append((0.0, p, V, 0.0))

    if not points:
        raise RuntimeError("No straight-line + hover data to fit")

    v_arr = np.array([p[0] for p in points])
    p_arr = np.array([p[1] for p in points])

    # Per-commanded-speed diagnostic table (grouping each commanded v_g's
    # trials together so a human reader can sanity-check that, e.g., v_g=10
    # actually flew at ~7.7 m/s in SITL). The sigma used for curve_fit
    # weighting is also derived from this grouping — points within the
    # same commanded-speed bucket share a power std-error.
    unique_v_cmd = sorted({pt[3] for pt in points})
    sigma = np.empty_like(p_arr)
    per_speed_diag = []
    for v_cmd in unique_v_cmd:
        mask = np.array([pt[3] == v_cmd for pt in points])
        n = int(mask.sum())
        mean_p = float(p_arr[mask].mean())
        std_p = float(p_arr[mask].std(ddof=1)) if n > 1 else 0.0
        se_p = std_p / math.sqrt(n) if n > 0 else 1.0
        sigma[mask] = max(se_p, 1.0)
        mean_v_meas = float(v_arr[mask].mean())
        mean_voltage = float(np.array(
            [pt[2] for pt in points if pt[3] == v_cmd]
        ).mean())
        per_speed_diag.append({
            "v_commanded": v_cmd,
            "v_mean_measured": mean_v_meas,
            "n": n,
            "mean_power_w": mean_p,
            "std_power_w": std_p,
            "mean_voltage_v": mean_voltage,
        })

    popt, pcov = scipy.optimize.curve_fit(
        _power_law, v_arr, p_arr, sigma=sigma, absolute_sigma=False,
        p0=[1.0, 1.0, max(np.mean(p_arr), 1.0)],
    )
    lambda_1, lambda_2, p_hover = popt
    one_sigma = np.sqrt(np.diag(pcov))

    # Goodness of fit
    pred = _power_law(v_arr, *popt)
    residuals = p_arr - pred
    ss_res = float((residuals ** 2).sum())
    ss_tot = float(((p_arr - p_arr.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    dof = len(v_arr) - 3
    chi2 = float((residuals ** 2 / sigma ** 2).sum())
    reduced_chi2 = chi2 / dof if dof > 0 else float("nan")

    hover_observed = float(np.array(
        [r["mean_power_w"] for r in hover_rows]
    ).astype(float).mean()) if hover_rows else float("nan")

    return {
        "lambda_1": float(lambda_1),
        "lambda_2": float(lambda_2),
        "P_hover": float(p_hover),
        "hover_power_observed_w": hover_observed,
        "covariance": pcov.tolist(),
        "one_sigma": {
            "lambda_1": float(one_sigma[0]),
            "lambda_2": float(one_sigma[1]),
            "P_hover": float(one_sigma[2]),
        },
        "R2": float(r2),
        "reduced_chi2": float(reduced_chi2),
        "fit_residual_rms_w": float(math.sqrt(ss_res / len(v_arr))),
        "per_speed_diagnostic": per_speed_diag,
    }


def fit_turn(
    turn_rows: List[Dict], straight_line_fit: Dict,
) -> Dict:
    """For each trial: E_turn = E_total - P_translate(v_g) * d_predicted / v_g.

    Then per (v_g, ω) group, linear regress E_turn vs delta_theta to get
    (E_acc, gamma).
    """
    l1 = straight_line_fit["lambda_1"]
    l2 = straight_line_fit["lambda_2"]
    p_h = straight_line_fit["P_hover"]

    def p_translate(v):
        return l1 * v * v + l2 * v + p_h

    # Group by (v_g, omega)
    groups: Dict[Tuple[float, float], List[Tuple[float, float]]] = {}
    for row in turn_rows:
        cond = row["condition"]
        v_g = float(cond["v_g_mps"])
        omega = float(cond["omega_rad_s"])
        delta = float(cond["delta_theta_rad"])
        d_pred = float(cond["predicted_straight_line_m"])
        hold_s = float(cond.get("hold_duration_s", 0.0))
        e_total = float(row["energy_j"])

        # Subtract the straight-line component based on the trial's actual
        # cruise speed (not the commanded v_g). The drone is stationary
        # during the j-hold, so the cruise portion of the snapshot covers
        # `distance_m` over `duration_s - hold_s`. PX4 SITL routinely caps
        # actual cruise below commanded v_g; using v_cruise_actual makes
        # the subtraction physically correct in that case and unchanged
        # when v_cruise_actual ≈ v_g.
        distance_m = float(row["distance_m"])
        duration_s = float(row["duration_s"])
        cruise_time = max(duration_s - hold_s, 1e-6)
        v_cruise = distance_m / cruise_time
        e_predicted_straight = p_translate(v_cruise) * distance_m / max(v_cruise, 1e-6)
        e_turn = e_total - e_predicted_straight

        key = (v_g, omega)
        groups.setdefault(key, []).append((delta, e_turn))

    per_group = []
    e_accs = []
    gammas = []
    skipped = []
    for (v_g, omega), pairs in sorted(groups.items()):
        deltas = np.array([p[0] for p in pairs])
        e_turns = np.array([p[1] for p in pairs])
        n_distinct_deltas = len(set(deltas.tolist()))
        if n_distinct_deltas < 2:
            skipped.append({
                "v_g": float(v_g),
                "omega": float(omega),
                "n": int(len(pairs)),
                "n_distinct_deltas": n_distinct_deltas,
                "reason": (
                    "Need ≥2 distinct Δθ values to fit E_acc + γ·Δθ; only "
                    f"{n_distinct_deltas} survived the speed_check filter."
                ),
            })
            continue
        slope, intercept, r, _, stderr = scipy.stats.linregress(deltas, e_turns)
        per_group.append({
            "v_g": float(v_g),
            "omega": float(omega),
            "E_acc": float(intercept),
            "gamma": float(slope),
            "R2": float(r * r),
            "n": int(len(pairs)),
            "stderr": float(stderr),
        })
        e_accs.append(float(intercept))
        gammas.append(float(slope))

    if not e_accs:
        raise RuntimeError("Insufficient turn data for fit")

    e_acc_arr = np.array(e_accs)
    gamma_arr = np.array(gammas)

    return {
        "E_acc": float(e_acc_arr.mean()),
        "E_acc_std": float(e_acc_arr.std(ddof=1)) if len(e_acc_arr) > 1 else 0.0,
        "gamma": float(gamma_arr.mean()),
        "gamma_std": float(gamma_arr.std(ddof=1)) if len(gamma_arr) > 1 else 0.0,
        "gamma_relative_spread": float(
            gamma_arr.std(ddof=1) / gamma_arr.mean()
        ) if len(gamma_arr) > 1 and gamma_arr.mean() != 0 else 0.0,
        "per_group": per_group,
        "skipped_groups": skipped,
    }


def _make_p_translate_plot(
    straight_rows: List[Dict], hover_rows: List[Dict],
    fit: Dict, out_path: Path,
):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 5))
    sl_v = np.array([float(r["mean_speed_mps"]) for r in straight_rows])
    sl_p = np.array([float(r["mean_power_w"]) for r in straight_rows])
    sl_cmd = np.array([float(r["condition"]["speed_mps"]) for r in straight_rows])
    hv_p = np.array([float(r["mean_power_w"]) for r in hover_rows])
    if sl_v.size:
        ax.scatter(sl_v, sl_p, alpha=0.6, label="straight-line trials (measured v)",
                   color="tab:blue")
        for v_meas, p_w, v_cmd in zip(sl_v, sl_p, sl_cmd):
            ax.annotate(f"cmd={v_cmd:.0f}", (v_meas, p_w), fontsize=7,
                        xytext=(3, 3), textcoords="offset points", alpha=0.6)
    if hv_p.size:
        ax.scatter(np.zeros_like(hv_p), hv_p, alpha=0.6, marker="s",
                   label="hover trials (v=0)", color="tab:orange")
    v_max = float(max(sl_v.max() if sl_v.size else 0.0, 1.0)) + 1.0
    v_grid = np.linspace(0, v_max, 100)
    p_pred = (
        fit["lambda_1"] * v_grid ** 2
        + fit["lambda_2"] * v_grid + fit["P_hover"]
    )
    ax.plot(v_grid, p_pred, "r-", linewidth=2, label="fit")
    ax.set_xlabel("measured airspeed v (m/s)")
    ax.set_ylabel("P_translate (W)")
    ax.set_title(
        f"P(v) = {fit['lambda_1']:.3f} v² + {fit['lambda_2']:.3f} v + {fit['P_hover']:.1f}"
        f"  (R² = {fit['R2']:.4f})"
    )
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def _make_hover_plot(hover_rows: List[Dict], fit: Dict, out_path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not hover_rows:
        return

    powers = np.array([float(r["mean_power_w"]) for r in hover_rows])
    voltages = np.array([float(r["mean_voltage_v"]) for r in hover_rows])
    durations = np.array([float(r["duration_s"]) for r in hover_rows])
    trials = np.arange(len(hover_rows))
    mean_p = float(powers.mean())
    std_p = float(powers.std(ddof=1)) if len(powers) > 1 else 0.0

    fig, (ax_p, ax_v) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    ax_p.scatter(trials, powers, color="tab:orange", label="trial mean power")
    ax_p.axhline(mean_p, color="tab:red", linestyle="-", linewidth=1.5,
                 label=f"mean = {mean_p:.2f} W")
    ax_p.axhline(fit["P_hover"], color="tab:blue", linestyle="--", linewidth=1.5,
                 label=f"P_hover (fit) = {fit['P_hover']:.2f} W")
    if std_p > 0:
        ax_p.axhspan(mean_p - std_p, mean_p + std_p, color="tab:red", alpha=0.10,
                     label=f"±1σ ({std_p:.2f} W)")
    ax_p.set_ylabel("mean power (W)")
    ax_p.set_title(
        f"Hover trials: mean = {mean_p:.2f} ± {std_p:.2f} W"
        f"  (n = {len(hover_rows)}, mean duration = {durations.mean():.1f} s)"
    )
    ax_p.legend(fontsize=8)
    ax_p.grid(True, alpha=0.3)

    ax_v.scatter(trials, voltages, color="tab:purple")
    ax_v.set_xlabel("trial index")
    ax_v.set_ylabel("mean battery voltage (V)")
    ax_v.grid(True, alpha=0.3)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def _make_e_turn_plot(
    turn_rows: List[Dict], straight_fit: Dict, turn_fit: Dict,
    out_path: Path,
):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    l1 = straight_fit["lambda_1"]
    l2 = straight_fit["lambda_2"]
    p_h = straight_fit["P_hover"]

    def p_translate(v):
        return l1 * v * v + l2 * v + p_h

    # Compute E_turn per row using the same formula fit_turn() uses, so the
    # scatter dots are consistent with the regression line.
    points: Dict[Tuple[float, float], List[Tuple[float, float]]] = {}
    for row in turn_rows:
        cond = row["condition"]
        v_g = float(cond["v_g_mps"])
        omega = float(cond["omega_rad_s"])
        delta = float(cond["delta_theta_rad"])
        hold_s = float(cond.get("hold_duration_s", 0.0))
        e_total = float(row["energy_j"])
        distance_m = float(row["distance_m"])
        duration_s = float(row["duration_s"])
        cruise_time = max(duration_s - hold_s, 1e-6)
        v_cruise = distance_m / cruise_time
        e_predicted_straight = (
            p_translate(v_cruise) * distance_m / max(v_cruise, 1e-6)
        )
        e_turn = e_total - e_predicted_straight
        points.setdefault((v_g, omega), []).append((delta, e_turn))

    keys = sorted(points.keys())
    n = len(keys)
    cols = 3
    rows_n = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows_n, cols, figsize=(15, 3.5 * rows_n),
                             squeeze=False)
    for i, key in enumerate(keys):
        r, c = divmod(i, cols)
        ax = axes[r][c]
        pairs = points[key]
        deltas = np.array([p[0] for p in pairs])
        e_turns = np.array([p[1] for p in pairs])
        ax.scatter(deltas, e_turns, alpha=0.5, color="tab:blue")
        # Find this group's fit
        grp = next(
            (g for g in turn_fit["per_group"]
             if abs(g["v_g"] - key[0]) < 1e-6
             and abs(g["omega"] - key[1]) < 1e-6),
            None,
        )
        if grp is not None:
            x = np.linspace(0, math.pi, 50)
            y = grp["E_acc"] + grp["gamma"] * x
            ax.plot(x, y, "r-", linewidth=2,
                    label=f"E_acc={grp['E_acc']:.1f}, γ={grp['gamma']:.1f}")
            ax.legend(fontsize=8)
        ax.set_xlabel("Δθ (rad)")
        ax.set_ylabel("E_turn (J)")
        ax.set_title(f"v_g={key[0]:.0f} m/s, ω={key[1]:.1f} rad/s")
        ax.grid(True, alpha=0.3)

    # Hide unused axes
    for j in range(n, rows_n * cols):
        r, c = divmod(j, cols)
        axes[r][c].axis("off")

    fig.suptitle(
        f"Turn cost: E_acc = {turn_fit['E_acc']:.1f} ± {turn_fit['E_acc_std']:.1f} J,"
        f" γ = {turn_fit['gamma']:.2f} ± {turn_fit['gamma_std']:.2f} J/rad",
    )
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def fit_all(
    data_dir: str, output_dir: str,
    px4_sitl_world: str = "empty",
) -> Dict:
    data_dir = Path(data_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    hover_rows = _read_csv_rows(data_dir / "hover.csv")
    straight_rows = _read_csv_rows(data_dir / "straight_line.csv")
    turn_rows = _read_csv_rows(data_dir / "turns.csv")

    # We don't drop trials based on passed_speed_check anymore. PX4 SITL's
    # trajectory smoother caps actual cruise below commanded at higher v_g,
    # so a non-trivial fraction of rows would otherwise be tossed.
    # - Straight-line fit uses mean_speed_mps (measured), so a "v=12" trial
    #   that actually flew 8.5 m/s is just one more (8.5, P) sample.
    # - Turn fit subtracts the straight-line component using the trial's
    #   actual cruise speed (extracted from distance_m / (duration_s -
    #   hold_duration_s)), so a capped trial still subtracts the right
    #   energy and contributes correct E_turn.
    # - Hover: expected_speed_mps=0 disables the speed check anyway.
    n_excluded_total = sum(
        1 for r in (straight_rows + turn_rows)
        if r["passed_speed_check"].lower() != "true"
    )

    straight_fit = fit_straight_line(straight_rows, hover_rows)
    turn_fit = fit_turn(turn_rows, straight_fit)

    constants = {
        "fit_metadata": {
            "generated_utc": _utc_iso(),
            "px4_sitl_world": px4_sitl_world,
            "num_straight_line_trials": len(straight_rows),
            "num_hover_trials": len(hover_rows),
            "num_turn_trials": len(turn_rows),
            "trials_excluded_by_speed_check": int(n_excluded_total),
        },
        "straight_line": straight_fit,
        "turn": turn_fit,
    }

    out_yaml = output_dir / "energy_constants.yaml"
    with out_yaml.open("w") as f:
        yaml.safe_dump(constants, f, sort_keys=False, default_flow_style=False)

    _make_p_translate_plot(
        straight_rows, hover_rows, straight_fit,
        output_dir / "plots" / "p_translate.png",
    )
    _make_hover_plot(
        hover_rows, straight_fit,
        output_dir / "plots" / "hover.png",
    )
    _make_e_turn_plot(
        turn_rows, straight_fit, turn_fit,
        output_dir / "plots" / "e_turn.png",
    )

    return constants


def _utc_iso() -> str:
    import datetime
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def main():
    parser = argparse.ArgumentParser(
        description="Fit energy constants from calibration CSVs",
    )
    parser.add_argument("--data-dir", required=True,
                        help="Directory containing hover.csv, straight_line.csv, turns.csv")
    parser.add_argument("--output-dir", required=True,
                        help="Where to write energy_constants.yaml + plots/")
    parser.add_argument("--px4-sitl-world", default="empty")
    args = parser.parse_args()

    constants = fit_all(
        data_dir=args.data_dir,
        output_dir=args.output_dir,
        px4_sitl_world=args.px4_sitl_world,
    )
    sl = constants["straight_line"]
    tu = constants["turn"]
    print(
        f"P(v) = {sl['lambda_1']:.4f} v² + {sl['lambda_2']:.4f} v + {sl['P_hover']:.2f}"
        f"  (R² = {sl['R2']:.4f}, fit RMS = {sl['fit_residual_rms_w']:.2f} W)"
    )
    print(
        f"E_acc = {tu['E_acc']:.2f} ± {tu['E_acc_std']:.2f} J,"
        f" γ = {tu['gamma']:.2f} ± {tu['gamma_std']:.2f} J/rad"
        f" (rel spread {tu['gamma_relative_spread']:.3f})"
    )


if __name__ == "__main__":
    main()
