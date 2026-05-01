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

    Each trial contributes one point: (v, mean_power_w). Hover trials are
    treated as v=0 points using their mean_power_w.
    """
    points: List[Tuple[float, float, float, float]] = []  # (v, P, V_mean, n)
    for row in straight_rows:
        v = float(row["condition"]["speed_mps"])
        p = float(row["mean_power_w"])
        V = float(row["mean_voltage_v"])
        points.append((v, p, V, 1))
    for row in hover_rows:
        p = float(row["mean_power_w"])
        V = float(row["mean_voltage_v"])
        points.append((0.0, p, V, 1))

    if not points:
        raise RuntimeError("No straight-line + hover data to fit")

    v_arr = np.array([p[0] for p in points])
    p_arr = np.array([p[1] for p in points])

    # Compute per-speed mean and std for sigma weighting.
    unique_v = sorted(set(v_arr.tolist()))
    sigma = np.empty_like(p_arr)
    per_speed_diag = []
    for v in unique_v:
        mask = v_arr == v
        n = int(mask.sum())
        mean_p = float(p_arr[mask].mean())
        std_p = float(p_arr[mask].std(ddof=1)) if n > 1 else 0.0
        # SE = std / sqrt(n)
        se_p = std_p / math.sqrt(n) if n > 0 else 1.0
        # Avoid zero sigma (would make curve_fit unstable).
        sigma[mask] = max(se_p, 1.0)
        mean_v = float(np.array([pt[2] for pt in points if pt[0] == v]).mean())
        per_speed_diag.append({
            "v": v, "n": n, "mean_power_w": mean_p, "std_power_w": std_p,
            "mean_voltage_v": mean_v,
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
        e_total = float(row["energy_j"])

        # The model's predicted straight-line cost (per the description).
        e_predicted_straight = p_translate(v_g) * d_pred / max(v_g, 1e-6)
        e_turn = e_total - e_predicted_straight

        key = (v_g, omega)
        groups.setdefault(key, []).append((delta, e_turn))

    per_group = []
    e_accs = []
    gammas = []
    for (v_g, omega), pairs in sorted(groups.items()):
        if len(pairs) < 2:
            continue
        deltas = np.array([p[0] for p in pairs])
        e_turns = np.array([p[1] for p in pairs])
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
    }


def _make_p_translate_plot(
    straight_rows: List[Dict], hover_rows: List[Dict],
    fit: Dict, out_path: Path,
):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 5))
    points = []
    for row in straight_rows:
        points.append((float(row["condition"]["speed_mps"]),
                       float(row["mean_power_w"])))
    for row in hover_rows:
        points.append((0.0, float(row["mean_power_w"])))
    if points:
        v = np.array([p[0] for p in points])
        p = np.array([p[1] for p in points])
        ax.scatter(v, p, alpha=0.5, label="trials", color="tab:blue")
    v_grid = np.linspace(0, 11, 100)
    p_pred = (
        fit["lambda_1"] * v_grid ** 2
        + fit["lambda_2"] * v_grid + fit["P_hover"]
    )
    ax.plot(v_grid, p_pred, "r-", linewidth=2, label="fit")
    ax.set_xlabel("airspeed v (m/s)")
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

    # Compute E_turn per row
    points: Dict[Tuple[float, float], List[Tuple[float, float]]] = {}
    for row in turn_rows:
        cond = row["condition"]
        v_g = float(cond["v_g_mps"])
        omega = float(cond["omega_rad_s"])
        delta = float(cond["delta_theta_rad"])
        d_pred = float(cond["predicted_straight_line_m"])
        e_total = float(row["energy_j"])
        e_turn = e_total - p_translate(v_g) * d_pred / max(v_g, 1e-6)
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

    n_excluded_total = 0
    straight_rows, n_ex = _filter_passed(straight_rows)
    n_excluded_total += n_ex
    turn_rows, n_ex = _filter_passed(turn_rows)
    n_excluded_total += n_ex
    # Hover trials have expected_speed_mps=0 → speed check disabled, all kept.

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
