"""Append adaptive-controller metrics from frozen, already aggregated exports.

This optional maintainer command does not read ROS bags or per-timestep logs.
Public reproduction uses the released CSVs and does not require these inputs.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
METHODS = {"slotine_li": "Slotine-Li", "ddp_gp": "DDP+GP"}
TRAJECTORIES = {
    "circle_plan": "Circular joint-space motion",
    "circle_xy": "Circle in the XY plane",
    "circle_yz": "Circle in the YZ plane",
    "ellipse_yz": "Ellipse in the YZ plane",
    "figure8_xz": "Figure-eight in the XZ plane",
    "figure8_xz_large": "Large figure-eight in the XZ plane",
    "joint_sine": "Joint-space sinusoidal motion",
    "line_x": "Line motion along X",
    "line_x_large": "Large line motion along X",
    "line_y": "Line motion along Y",
    "line_y_large": "Large line motion along Y",
    "line_z": "Line motion along Z",
    "orientation_swing": "Orientation swing",
    "reach_far_front": "Far frontal reach",
    "reach_far_left_high": "Far upper-left reach",
    "reach_far_right_low": "Far lower-right reach",
    "reach_front": "Frontal reach",
    "reach_left_high": "Upper-left reach",
    "reach_right_low": "Lower-right reach",
    "test_joint_chirp": "Joint chirp",
    "test_lissajous_xy": "Horizontal Lissajous motion",
    "test_orientation_micro_translation": "Orientation with a small translation",
    "test_reach_diagonal_front_high": "Diagonal reach toward a high frontal target",
    "test_tilted_circle_small": "Small tilted circle",
}
TRAJECTORIES = {"panda_ddp_" + key: value for key, value in TRAJECTORIES.items()}
TRACKING_FIELDS = {
    "ee_position_mean_mm": "ee_mean_mm_mean",
    "ee_position_max_mm": "ee_max_mm_mean",
    "ee_orientation_mean_deg": "ori_mean_deg_mean",
    "joint_position_rms_deg": "q_rms_mean_deg_mean",
    "joint_velocity_rms_deg_s": "dq_rms_mean_deg_s_mean",
}
TORQUE_FIELDS = {
    "measured_torque_rate_rms_nm_s": "torque_rate_rms_Nm_s",
    "measured_torque_rate_p95_nm_s": "abs_torque_rate_p95_Nm_s",
    "measured_torque_rate_p99_nm_s": "abs_torque_rate_p99_Nm_s",
    "measured_torque_rate_peak_nm_s": "abs_torque_rate_peak_Nm_s",
    "measured_torque_rms_nm": "torque_rms_Nm",
}


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def write_rows(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), quoting=csv.QUOTE_ALL)
        writer.writeheader()
        writer.writerows(rows)


def checked_value(row: dict[str, str], field: str) -> str:
    value = row[field]
    if not math.isfinite(float(value)) or float(value) < 0:
        raise ValueError(f"Invalid {field}: {value}")
    return value


def import_summaries(analysis_root: Path, torque_summary: Path) -> None:
    tracking_path = ROOT / "hardware/tracking_and_torque/per_trajectory_metrics.csv"
    payload_path = ROOT / "hardware/payload_robustness/per_trajectory_metrics.csv"
    tracking = [r for r in read_rows(tracking_path) if r["method"] not in METHODS.values()]
    payload = [r for r in read_rows(payload_path) if r["method"] not in METHODS.values()]
    original_tracking = [dict(r) for r in tracking]
    original_payload = [dict(r) for r in payload]
    torque_rows = read_rows(torque_summary)
    torque = {(r["set"], r["method"], r["trajectory"]): r for r in torque_rows}
    if len(torque) != len(torque_rows):
        raise ValueError("Duplicate measured-torque summary keys")
    expected_tasks = {
        "regular": {k for k in TRAJECTORIES if "_test_" not in k},
        "test": {k for k in TRAJECTORIES if "_test_" in k},
    }
    for grams in (0, 200, 600, 1000):
        rows = read_rows(analysis_root / f"analysis_20261118_all_{grams}g/per_trajectory.csv")
        for internal_method, method in METHODS.items():
            for cohort, set_name in (("regular", "Training"), ("test", "Held-out")):
                selected = [r for r in rows if r["method"] == internal_method and r["cohort"] == cohort]
                if len(selected) != len(expected_tasks[cohort]) or {r["traj"] for r in selected} != expected_tasks[cohort]:
                    raise ValueError(f"Unmatched trajectory set: {grams}, {method}, {cohort}")
                for row in selected:
                    repeats = 1 if grams else (6 if internal_method == "ddp_gp" and row["traj"] == "panda_ddp_test_joint_chirp" else 3)
                    if int(row["n_runs"]) != repeats or float(row["payload_g"]) != grams:
                        raise ValueError(f"Unexpected repeat count or payload: {row['traj']}")
                    if grams == 0:
                        torque_row = torque[set_name, method, row["traj"]]
                        if int(torque_row["n_runs"]) != repeats:
                            raise ValueError("Tracking/torque repeat counts differ")
                        record = {"set": set_name, "method": method, "trajectory": TRAJECTORIES[row["traj"]]}
                        record.update({k: checked_value(row, v) for k, v in TRACKING_FIELDS.items()})
                        record.update({k: checked_value(torque_row, v) for k, v in TORQUE_FIELDS.items()})
                        tracking.append(record)
                    if cohort == "regular":
                        payload.append({
                            "method": method, "payload_kg": str(grams / 1000),
                            "trajectory": TRAJECTORIES[row["traj"]],
                            **{k: checked_value(row, TRACKING_FIELDS[k]) for k in (
                                "ee_position_mean_mm", "ee_orientation_mean_deg", "joint_position_rms_deg")},
                        })
    if len(tracking) != 144 or len(payload) != 380:
        raise ValueError("Unexpected release record counts")
    assert tracking[:len(original_tracking)] == original_tracking
    assert payload[:len(original_payload)] == original_payload
    write_rows(tracking_path, tracking)
    write_rows(payload_path, payload)
    print("Imported 48 tracking/torque rows and 152 payload rows; existing rows retained.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis-root", type=Path, required=True)
    parser.add_argument("--torque-summary", type=Path, required=True)
    args = parser.parse_args()
    import_summaries(args.analysis_root, args.torque_summary)
