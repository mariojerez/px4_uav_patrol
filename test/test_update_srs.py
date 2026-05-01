"""Unit tests for update_srs.py: idempotency and correct placeholder
substitution."""

import shutil
import sys
import tempfile
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
CALIBRATION_DIR = HERE.parent / "calibration"
sys.path.insert(0, str(CALIBRATION_DIR))

import update_srs  # noqa: E402


SAMPLE_SRS = r"""
# FarmGuard SRS Excerpt

After curve fitting the data, $\lambda_1, \lambda_2 = \texttt{enter value once experiments finish}$ and $P_{hover} = \texttt{insert number}$ watts.

Through these experiments, we derived $E_{acc} = \texttt{enter num}$ J and $\gamma = \texttt{enter num}$ J/rad.

The experiment-parameter table (Table \ref{tab:experiment-parameters}) lists every $(v_g, \omega)$ combination swept.
"""


SAMPLE_CONSTANTS = {
    "fit_metadata": {
        "generated_utc": "2026-04-30T00:00:00Z",
        "px4_sitl_world": "empty",
        "num_straight_line_trials": 35,
        "num_hover_trials": 7,
        "num_turn_trials": 315,
        "trials_excluded_by_speed_check": 0,
    },
    "straight_line": {
        "lambda_1": 0.0512,
        "lambda_2": 0.2934,
        "P_hover": 184.21,
        "hover_power_observed_w": 184.05,
        "covariance": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "one_sigma": {"lambda_1": 0.001, "lambda_2": 0.01, "P_hover": 0.5},
        "R2": 0.992,
        "reduced_chi2": 1.05,
        "fit_residual_rms_w": 0.5,
        "per_speed_diagnostic": [],
    },
    "turn": {
        "E_acc": 211.4,
        "E_acc_std": 5.2,
        "gamma": 49.7,
        "gamma_std": 1.8,
        "gamma_relative_spread": 0.036,
        "per_group": [
            {"v_g": 2.0, "omega": 0.5, "E_acc": 210.0, "gamma": 49.5, "R2": 0.99, "n": 35},
            {"v_g": 2.0, "omega": 1.0, "E_acc": 211.0, "gamma": 49.8, "R2": 0.99, "n": 35},
            {"v_g": 4.0, "omega": 1.0, "E_acc": 212.5, "gamma": 50.1, "R2": 0.98, "n": 35},
        ],
    },
}


def _setup(tmp: Path) -> tuple:
    srs_path = tmp / "srs.md"
    constants_path = tmp / "energy_constants.yaml"
    srs_path.write_text(SAMPLE_SRS)
    with constants_path.open("w") as f:
        yaml.safe_dump(SAMPLE_CONSTANTS, f)
    return srs_path, constants_path


def test_replaces_lambda_placeholder():
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        srs_path, constants_path = _setup(tmp)
        update_srs.update_srs(str(srs_path), str(constants_path))
        text = srs_path.read_text()
        assert "0.0512, 0.2934" in text
        assert "enter value once experiments finish" not in text


def test_replaces_p_hover_placeholder():
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        srs_path, constants_path = _setup(tmp)
        update_srs.update_srs(str(srs_path), str(constants_path))
        text = srs_path.read_text()
        assert "P_{hover} = 184.21" in text
        assert "insert number" not in text


def test_replaces_eacc_and_gamma_placeholders():
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        srs_path, constants_path = _setup(tmp)
        update_srs.update_srs(str(srs_path), str(constants_path))
        text = srs_path.read_text()
        assert "E_{acc} = 211.40" in text
        assert "\\gamma = 49.70" in text
        assert "enter num" not in text


def test_inserts_experiment_table():
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        srs_path, constants_path = _setup(tmp)
        update_srs.update_srs(str(srs_path), str(constants_path))
        text = srs_path.read_text()
        assert update_srs.EXPERIMENT_TABLE_BEGIN in text
        assert update_srs.EXPERIMENT_TABLE_END in text
        # Check the table has the right rows.
        assert "| 2.0 | 0.5 |" in text
        assert "| 4.0 | 1.0 |" in text


def test_idempotent():
    """Running update twice produces an identical file."""
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        srs_path, constants_path = _setup(tmp)
        update_srs.update_srs(str(srs_path), str(constants_path))
        first = srs_path.read_text()
        update_srs.update_srs(str(srs_path), str(constants_path))
        second = srs_path.read_text()
        assert first == second


def test_dry_run_does_not_write():
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        srs_path, constants_path = _setup(tmp)
        original = srs_path.read_text()
        update_srs.update_srs(str(srs_path), str(constants_path), dry_run=True)
        assert srs_path.read_text() == original
