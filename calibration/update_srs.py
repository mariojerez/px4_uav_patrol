"""Idempotently fill the energy-cost placeholders in srs.md from a fitted
`energy_constants.yaml`.

The four target phrases (added to srs.md §7.7.1) are:

  \\lambda_1, \\lambda_2 = \\texttt{enter value once experiments finish}
  P_{hover} = \\texttt{insert number}
  E_{acc} = \\texttt{enter num}
  \\gamma = \\texttt{enter num}

Each placeholder has a unique surrounding context so we use literal regex
replacement against the raw markdown. The replacements are idempotent: re-
running with new fitted values rewrites the same spans, so the script can
be run repeatedly as the calibration is refined.

Also populates Table \\ref{tab:experiment-parameters} with the actual
(v_g, omega) combinations swept in the run.
"""

import argparse
import re
from pathlib import Path

import yaml


# Regex patterns matching either the original placeholder OR a previously-
# filled value. Re-running the script should be a no-op when the constants
# are unchanged.
LAMBDA_PATTERN = re.compile(
    r"\\lambda_1, \\lambda_2 = (?:"
    r"\\texttt\{enter value once experiments finish\}"
    r"|"
    r"[-0-9.eE+]+, [-0-9.eE+]+"
    r")"
)
P_HOVER_PATTERN = re.compile(
    r"P_\{hover\} = (?:"
    r"\\texttt\{insert number\}"
    r"|"
    r"[-0-9.eE+]+"
    r")"
)
E_ACC_PATTERN = re.compile(
    r"E_\{acc\} = (?:"
    r"\\texttt\{enter num\}"
    r"|"
    r"[-0-9.eE+]+"
    r")"
)
GAMMA_PATTERN = re.compile(
    r"\\gamma = (?:"
    r"\\texttt\{enter num\}"
    r"|"
    r"[-0-9.eE+]+"
    r")"
)

EXPERIMENT_TABLE_BEGIN = "<!-- BEGIN tab:experiment-parameters -->"
EXPERIMENT_TABLE_END = "<!-- END tab:experiment-parameters -->"


def _format_table(per_group: list) -> str:
    """Build a Markdown table block summarizing each (v_g, omega) group."""
    lines = []
    lines.append(EXPERIMENT_TABLE_BEGIN)
    lines.append("")
    lines.append("| v_g (m/s) | ω (rad/s) | E_acc (J) | γ (J/rad) | R² | n |")
    lines.append("|---|---|---|---|---|---|")
    for g in sorted(per_group, key=lambda r: (r["v_g"], r["omega"])):
        lines.append(
            f"| {g['v_g']:.1f} | {g['omega']:.1f} | "
            f"{g['E_acc']:.2f} | {g['gamma']:.2f} | "
            f"{g['R2']:.4f} | {g['n']} |"
        )
    lines.append("")
    lines.append(EXPERIMENT_TABLE_END)
    return "\n".join(lines)


def _replace_or_insert_table(content: str, table_block: str) -> str:
    """Replace any existing tab:experiment-parameters block, or insert a new
    one immediately after the line that references it for the first time.
    """
    block_re = re.compile(
        re.escape(EXPERIMENT_TABLE_BEGIN) + r".*?" + re.escape(EXPERIMENT_TABLE_END),
        re.DOTALL,
    )
    if block_re.search(content):
        return block_re.sub(table_block, content)
    # Insert after the first reference to tab:experiment-parameters
    ref_re = re.compile(r"(Table \\ref\{tab:experiment-parameters\}[^\n]*\n)")
    m = ref_re.search(content)
    if m is None:
        return content
    return content[:m.end()] + "\n" + table_block + "\n" + content[m.end():]


def update_srs(srs_path: str, constants_path: str, dry_run: bool = False) -> str:
    srs_path = Path(srs_path)
    constants_path = Path(constants_path)
    if not srs_path.exists():
        raise FileNotFoundError(f"srs.md not found at {srs_path}")
    if not constants_path.exists():
        raise FileNotFoundError(f"energy_constants.yaml not found at {constants_path}")

    with constants_path.open() as f:
        c = yaml.safe_load(f)
    sl = c["straight_line"]
    tu = c["turn"]

    text = srs_path.read_text()
    new_text = LAMBDA_PATTERN.sub(
        f"\\\\lambda_1, \\\\lambda_2 = {sl['lambda_1']:.4f}, {sl['lambda_2']:.4f}",
        text,
    )
    new_text = P_HOVER_PATTERN.sub(
        f"P_{{hover}} = {sl['P_hover']:.2f}",
        new_text,
    )
    new_text = E_ACC_PATTERN.sub(
        f"E_{{acc}} = {tu['E_acc']:.2f}",
        new_text,
    )
    new_text = GAMMA_PATTERN.sub(
        f"\\\\gamma = {tu['gamma']:.2f}",
        new_text,
    )

    table_block = _format_table(tu["per_group"])
    new_text = _replace_or_insert_table(new_text, table_block)

    if not dry_run:
        srs_path.write_text(new_text)
    return new_text


def main():
    parser = argparse.ArgumentParser(
        description="Fill the SRS energy-cost placeholders from energy_constants.yaml",
    )
    parser.add_argument("--srs", default="/app/srs.md")
    parser.add_argument(
        "--constants",
        required=True,
        help="Path to energy_constants.yaml from fit_energy_constants.py",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print the result instead of writing to disk",
    )
    args = parser.parse_args()
    new = update_srs(args.srs, args.constants, dry_run=args.dry_run)
    if args.dry_run:
        print(new)
    else:
        print(f"Updated {args.srs} with constants from {args.constants}")


if __name__ == "__main__":
    main()
