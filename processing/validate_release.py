"""Validate the public data package against the manuscript tables."""

from __future__ import annotations

from pathlib import Path
import hashlib
import json
import re

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "reproduced" / "tables"


def assert_close(actual: float, expected: float, label: str, atol: float) -> None:
    if not np.isclose(float(actual), float(expected), rtol=0.0, atol=atol):
        raise AssertionError(f"{label}: actual={actual}, expected={expected}")


def validate_source_files() -> None:
    expected_rows = {
        "simulation/learning_efficiency/learning_curves.csv": 1087,
        "simulation/controller_comparison/evaluation_metrics.csv": 9,
        "simulation/component_ablation/evaluation_metrics.csv": 15,
        "simulation/component_ablation/learning_curves.csv": 500,
        "hardware/tracking_and_torque/per_trajectory_metrics.csv": 144,
        "hardware/dual_endpoint/per_trajectory_metrics.csv": 48,
        "hardware/payload_robustness/per_trajectory_metrics.csv": 380,
    }
    expected_columns = {
        "simulation/learning_efficiency/learning_curves.csv": [
            "method", "training_transitions", "mean_evaluation_return"
        ],
        "simulation/controller_comparison/evaluation_metrics.csv": [
            "method", "evaluation", "training_transitions", "mean_step_reward",
            "joint_position_rms_deg", "ee_position_rms_mm",
            "ee_orientation_rms_deg",
        ],
        "simulation/component_ablation/learning_curves.csv": [
            "method", "training_transitions", "mean_evaluation_return"
        ],
        "simulation/component_ablation/evaluation_metrics.csv": [
            "method", "evaluation", "mean_step_reward",
            "joint_position_rms_deg", "joint_velocity_rms_deg_s",
            "ee_position_rms_mm", "ee_orientation_rms_deg",
            "residual_torque_rms_nm", "residual_torque_rate_rms_nm_s",
        ],
        "hardware/tracking_and_torque/per_trajectory_metrics.csv": [
            "set", "method", "trajectory", "ee_position_mean_mm",
            "ee_position_max_mm", "ee_orientation_mean_deg",
            "joint_position_rms_deg", "joint_velocity_rms_deg_s",
            "measured_torque_rate_rms_nm_s", "measured_torque_rate_p95_nm_s",
            "measured_torque_rate_p99_nm_s", "measured_torque_rate_peak_nm_s",
            "measured_torque_rms_nm",
        ],
        "hardware/dual_endpoint/per_trajectory_metrics.csv": [
            "set", "method", "trajectory", "ee_position_mean_mm",
            "joint_position_rms_deg", "joint_velocity_rms_deg_s",
            "commanded_torque_rate_rms_nm_s",
            "measured_torque_rate_rms_nm_s",
            "measured_torque_second_derivative_rms_nm_s2",
        ],
        "hardware/payload_robustness/per_trajectory_metrics.csv": [
            "method", "payload_kg", "trajectory", "ee_position_mean_mm",
            "ee_orientation_mean_deg", "joint_position_rms_deg",
        ],
    }
    for relative, expected in expected_rows.items():
        data = pd.read_csv(ROOT / relative)
        actual = len(data)
        if actual != expected:
            raise AssertionError(f"{relative}: {actual} rows, expected {expected}")
        if list(data.columns) != expected_columns[relative]:
            raise AssertionError(f"Unexpected columns in {relative}")
        if data.isna().any().any():
            raise AssertionError(f"Missing value in {relative}")

    key_columns = {
        "simulation/learning_efficiency/learning_curves.csv": [
            "method", "training_transitions"
        ],
        "simulation/controller_comparison/evaluation_metrics.csv": [
            "method", "evaluation"
        ],
        "simulation/component_ablation/learning_curves.csv": [
            "method", "training_transitions"
        ],
        "simulation/component_ablation/evaluation_metrics.csv": [
            "method", "evaluation"
        ],
        "hardware/tracking_and_torque/per_trajectory_metrics.csv": [
            "set", "method", "trajectory"
        ],
        "hardware/dual_endpoint/per_trajectory_metrics.csv": [
            "set", "method", "trajectory"
        ],
        "hardware/payload_robustness/per_trajectory_metrics.csv": [
            "method", "payload_kg", "trajectory"
        ],
    }
    for relative, keys in key_columns.items():
        data = pd.read_csv(ROOT / relative)
        if data.duplicated(keys).any():
            raise AssertionError(f"Duplicate record in {relative}")
        numeric = data.select_dtypes(include=[np.number])
        if not np.isfinite(numeric.to_numpy()).all():
            raise AssertionError(f"Non-finite value in {relative}")

    learning = pd.read_csv(
        ROOT / "simulation/learning_efficiency/learning_curves.csv"
    )
    if set(learning["method"]) != {"Direct RL", "Proposed"}:
        raise AssertionError("Unexpected methods in Fig. 1 data")
    if learning["training_transitions"].min() <= 0:
        raise AssertionError("Training-transition values must be positive")

    repeated_evaluations = (
        "simulation/controller_comparison/evaluation_metrics.csv",
        "simulation/component_ablation/evaluation_metrics.csv",
    )
    for relative in repeated_evaluations:
        data = pd.read_csv(ROOT / relative)
        for method, block in data.groupby("method"):
            if set(block["evaluation"]) != {1, 2, 3}:
                raise AssertionError(f"Unexpected evaluations for {method}")

    tracking = pd.read_csv(
        ROOT / "hardware/tracking_and_torque/per_trajectory_metrics.csv"
    )
    dual_endpoint = pd.read_csv(
        ROOT / "hardware/dual_endpoint/per_trajectory_metrics.csv"
    )
    hardware_methods = {"Nominal DDP", "Friction DDP", "Slotine-Li", "DDP+GP",
                        "Joint velocity control", "Proposed"}
    if set(tracking["method"]) != hardware_methods:
        raise AssertionError("Unexpected Fig. 4 controller set")
    if set(dual_endpoint["method"]) != {"ZOH residual", "Proposed"}:
        raise AssertionError("Unexpected Table V controller set")
    for data in (tracking, dual_endpoint):
        if set(data["set"]) != {"Training", "Held-out"}:
            raise AssertionError("Unexpected hardware evaluation sets")
        counts = data.groupby(["set", "method"]).size()
        for (set_name, method), count in counts.items():
            expected = 19 if set_name == "Training" else 5
            if count != expected:
                raise AssertionError(
                    f"Unexpected trajectory count for {set_name}, {method}"
                )
        for set_name, block in data.groupby("set"):
            reference_tasks = set(tracking[(tracking["set"] == set_name) &
                                           (tracking["method"] == "Proposed")]["trajectory"])
            for method, method_rows in block.groupby("method"):
                if set(method_rows["trajectory"]) != reference_tasks:
                    raise AssertionError(f"Unmatched trajectories: {set_name}, {method}")

    payload = pd.read_csv(
        ROOT / "hardware/payload_robustness/per_trajectory_metrics.csv"
    )
    payload_counts = payload.groupby(["method", "payload_kg"]).size()
    payload_methods = hardware_methods - {"Joint velocity control"}
    expected_payload_groups = {(method, mass) for method in payload_methods for mass in (0., .2, .6, 1.)}
    if set(payload_counts.index) != expected_payload_groups:
        raise AssertionError("Unexpected payload method/mass groups")
    if not (payload_counts == 19).all():
        raise AssertionError("Unexpected payload trajectory count")
    reference_tasks = set(tracking[(tracking["set"] == "Training") &
                                   (tracking["method"] == "Proposed")]["trajectory"])
    for group, block in payload.groupby(["method", "payload_kg"]):
        if set(block["trajectory"]) != reference_tasks:
            raise AssertionError(f"Unmatched payload trajectories: {group}")

    allowed_methods = {
        "Nominal DDP",
        "Direct RL",
        "Friction DDP",
        "Slotine-Li",
        "DDP+GP",
        "Joint velocity control",
        "ZOH residual",
        "MLP baseline",
        "MLP + W.",
        "Rec. + W.",
        "Rec. + W. + ID",
        "Proposed",
    }
    for path in ROOT.rglob("*.csv"):
        data = pd.read_csv(path)
        if "method" in data.columns:
            documented = allowed_methods
            if path.parent == ROOT / "hardware/circular_trajectory":
                documented = {"pure_ddp_no_rl", "friction", "slotine_li", "ddp_gp",
                              "velocity_control", "ab05_residual_rl"}
            unknown = set(data["method"].dropna()) - documented
            if unknown:
                raise AssertionError(f"Undocumented method names in {path}: {unknown}")


def validate_table_ii() -> None:
    actual = pd.read_csv(TABLES / "table_ii_simulation_controller_comparison.csv")
    expected = {
        "Nominal DDP": (2.42296, 0.58668, 0.84715, 0.68724),
        "Direct RL": (2.71734, 0.22675, 1.58435, 0.23330),
        "Proposed": (3.56400, 0.05781, 0.23307, 0.07557),
    }
    columns = [
        "mean_step_reward_mean",
        "joint_position_rms_deg_mean",
        "ee_position_rms_mm_mean",
        "ee_orientation_rms_deg_mean",
    ]
    for method, values in expected.items():
        row = actual[actual["method"] == method].iloc[0]
        for column, value in zip(columns, values):
            assert_close(row[column], value, f"Table II {method} {column}", 5e-6)
    expected_std = {
        "Nominal DDP": (0.00130, 0.00011, 0.00283, 0.00029),
        "Direct RL": (0.00145, 0.00007, 0.00358, 0.00245),
        "Proposed": (0.00104, 0.00063, 0.00152, 0.00063),
    }
    for method, values in expected_std.items():
        row = actual[actual["method"] == method].iloc[0]
        for column, value in zip(columns, values):
            std_column = column.removesuffix("_mean") + "_std"
            assert_close(row[std_column], value, f"Table II {method} {std_column}", 5e-6)


def validate_table_iv() -> None:
    actual = pd.read_csv(TABLES / "table_iv_simulation_component_ablation.csv")
    expected = {
        "MLP baseline": (3.348, 0.0576, 0.867, 0.304, 0.0898, 0.568, 23.38),
        "MLP + W.": (3.356, 0.0584, 0.918, 0.273, 0.0943, 0.595, 52.05),
        "Rec. + W.": (3.411, 0.0536, 0.770, 0.237, 0.0916, 0.597, 47.08),
        "Rec. + W. + ID": (3.446, 0.0485, 0.702, 0.262, 0.0897, 0.584, 35.58),
        "Proposed": (3.429, 0.0467, 0.674, 0.210, 0.0756, 0.578, 45.40),
    }
    columns = [
        "mean_step_reward_mean",
        "joint_position_rms_deg_mean",
        "joint_velocity_rms_deg_s_mean",
        "ee_position_rms_mm_mean",
        "ee_orientation_rms_deg_mean",
        "residual_torque_rms_nm_mean",
        "residual_torque_rate_rms_nm_s_mean",
    ]
    for method, values in expected.items():
        row = actual[actual["method"] == method].iloc[0]
        for column, value in zip(columns, values):
            tolerance = 5e-3 if abs(value) >= 10 else 5e-4
            assert_close(row[column], value, f"Table IV {method} {column}", tolerance)


def validate_figure_4_summary() -> None:
    actual = pd.read_csv(TABLES / "figure_4_tracking_torque_summary.csv")
    expected = {
        ("Training", "Nominal DDP"): (1.379, 2.419, 1.811, 1.014, 4.204, 35.993),
        ("Training", "Friction DDP"): (1.007, 2.207, 1.307, .774, 4.195, 34.916),
        ("Training", "Slotine-Li"): (.188, .848, 1.726, .366, 4.196, 36.432),
        ("Training", "DDP+GP"): (.405, 2.842, .561, .343, 4.221, 36.765),
        ("Training", "Joint velocity control"): (.132, .433, .431, .228, 4.182, 38.002),
        ("Training", "Proposed"): (.107, 2.640, .331, .189, 4.237, 43.896),
        ("Held-out", "Nominal DDP"): (1.216, 2.422, 2.565, 1.348, 4.009, 28.450),
        ("Held-out", "Friction DDP"): (.968, 2.302, 1.962, 1.184, 3.976, 28.102),
        ("Held-out", "Slotine-Li"): (.201, .839, 1.729, .419, 3.971, 29.512),
        ("Held-out", "DDP+GP"): (.408, 2.770, .736, .550, 3.985, 30.343),
        ("Held-out", "Joint velocity control"): (.128, .425, .351, .224, 4.021, 31.552),
        ("Held-out", "Proposed"): (.110, 2.678, .338, .203, 4.038, 37.755),
    }
    columns = [
        "joint_position_rms_deg_mean",
        "joint_velocity_rms_deg_s_mean",
        "ee_position_mean_mm_mean",
        "ee_orientation_mean_deg_mean",
        "measured_torque_rms_nm_mean",
        "measured_torque_rate_rms_nm_s_mean",
    ]
    if len(actual) != 12 or actual.duplicated(["set", "method"]).any():
        raise AssertionError("Unexpected Fig. 4 summary rows")
    source = pd.read_csv(ROOT / "hardware/tracking_and_torque/per_trajectory_metrics.csv")
    for key, values in expected.items():
        row = actual[(actual["set"] == key[0]) & (actual["method"] == key[1])].iloc[0]
        samples = source[(source["set"] == key[0]) & (source["method"] == key[1])]
        for column, value in zip(columns, values):
            assert_close(row[column], value, f"Fig. 4 {key} {column}", 5e-4)
            metric = column.removesuffix("_mean")
            assert_close(row[column], samples[metric].mean(), f"Fig. 4 mean {key} {metric}", 1e-12)
            assert_close(row[metric + "_std"], samples[metric].std(ddof=1),
                         f"Fig. 4 between-trajectory SD {key} {metric}", 1e-12)


def validate_payload() -> None:
    """Check all twenty plotted points against the frozen paper aggregation."""
    actual = pd.read_csv(TABLES / "figure_6_payload_summary.csv")
    expected = {
        "Nominal DDP": [(1.811328158, 1.013603105, 1.378674752),
                        (2.357067737, 1.031935632, 1.386208414),
                        (3.822978000, 1.050338316, 1.397041278),
                        (5.634355158, 1.101098263, 1.454958579)],
        "Friction DDP": [(1.307411737, .774391789, 1.006533970),
                         (1.695258632, .713230526, 1.012518586),
                         (3.291663421, .766896789, 1.140160023),
                         (5.269448263, .824679526, 1.293786368)],
        "Slotine-Li": [(1.725852317, .366486575, .187952668),
                       (2.311764192, .428667236, .200668710),
                       (4.881833839, .743271288, .259819439),
                       (7.808189303, 1.155631032, .345845913)],
        "DDP+GP": [(.561024132, .342881873, .405194938),
                   (.612545446, .350206466, .420268466),
                   (.745802671, .353149880, .421251077),
                   (1.073376062, .370200476, .442116144)],
        "Proposed": [(.330701053, .189243211, .107168060),
                     (.424010211, .182627789, .108555534),
                     (.813837526, .195730263, .118916165),
                     (1.389811316, .232810684, .141948602)],
    }
    columns = ["ee_position_mean_mm", "ee_orientation_mean_deg", "joint_position_rms_deg"]
    if len(actual) != 20 or actual.duplicated(["method", "payload_kg"]).any():
        raise AssertionError("Unexpected payload figure summary rows")
    source = pd.read_csv(ROOT / "hardware/payload_robustness/per_trajectory_metrics.csv")
    for method, values_by_mass in expected.items():
        for mass, values in zip((0., .2, .6, 1.), values_by_mass):
            row = actual[(actual["method"] == method) & (actual["payload_kg"] == mass)].iloc[0]
            samples = source[(source["method"] == method) & (source["payload_kg"] == mass)]
            for column, value in zip(columns, values):
                assert_close(row[column], value, f"Fig. 6 {method} {mass} {column}", 1e-8)
                assert_close(row[column], samples[column].mean(),
                             f"Fig. 6 equal trajectory weighting {method} {mass} {column}", 1e-12)


def validate_table_v() -> None:
    actual = pd.read_csv(TABLES / "table_v_dual_endpoint_ablation.csv")
    expected = {
        ("Training", "ZOH residual"): (0.441, 0.132, 3.705, 52.537, 50.416, 882.579),
        ("Training", "Proposed"): (0.331, 0.107, 2.640, 39.899, 43.896, 548.352),
        ("Held-out", "ZOH residual"): (0.424, 0.132, 3.446, 46.711, 44.031, 798.450),
        ("Held-out", "Proposed"): (0.338, 0.110, 2.678, 36.273, 37.755, 501.488),
    }
    columns = [
        "ee_position_mean_mm_mean",
        "joint_position_rms_deg_mean",
        "joint_velocity_rms_deg_s_mean",
        "commanded_torque_rate_rms_nm_s_mean",
        "measured_torque_rate_rms_nm_s_mean",
        "measured_torque_second_derivative_rms_nm_s2_mean",
    ]
    for key, values in expected.items():
        row = actual[(actual["set"] == key[0]) & (actual["method"] == key[1])].iloc[0]
        for column, value in zip(columns, values):
            assert_close(row[column], value, f"Table V {key} {column}", 5e-4)


def validate_circular_trajectory() -> None:
    """Check Fig. 5 provenance, repeat means, and full-rate paper statistics."""
    folder = ROOT / "hardware/circular_trajectory"
    provenance = json.loads((folder / "provenance.json").read_text(encoding="utf-8"))
    for record in provenance["released_data"]:
        actual_hash = hashlib.sha256((folder / record["filename"]).read_bytes()).hexdigest()
        if actual_hash != record["sha256"] or actual_hash != record["source_sha256"]:
            raise AssertionError(f"Fig. 5 source hash mismatch: {record['filename']}")
    from plot_circular_trajectory import _load_curves
    _, curves = _load_curves()
    for method, curve in curves.items():
        actual = np.column_stack([curve[f"{axis}_m"] for axis in "xyz"])
        reference = np.column_stack([curve[f"{axis}_ref_m"] for axis in "xyz"])
        # Averaged scalar errors must bound the error of the averaged paths.
        if (np.linalg.norm(actual-reference, axis=1)*1000 > curve["ee_mm"] + 1e-8).any():
            raise AssertionError(f"Fig. 5 scalar-error averaging is inconsistent: {method}")
    stats = pd.read_csv(folder / "selected_case_metrics.csv")
    source = pd.read_csv(ROOT / "hardware/tracking_and_torque/per_trajectory_metrics.csv")
    source = source[(source["set"] == "Training") & (source["trajectory"] == "Circular joint-space motion")]
    for _, row in stats.iterrows():
        label = row["label"].replace("Slotine--Li", "Slotine-Li")
        reference = source[source["method"] == label]
        if len(reference) != 1:
            raise AssertionError(f"Missing Fig. 5 full-rate summary: {label}")
        for actual, expected in (("ee_mean_mm", "ee_position_mean_mm"),
                                 ("ori_mean_deg", "ee_orientation_mean_deg"),
                                 ("joint_position_rmse_deg", "joint_position_rms_deg")):
            assert_close(row[actual], reference.iloc[0][expected], f"Fig. 5 {label} {actual}", 1e-6)
    values = stats.set_index("method")["ee_mean_mm"]
    assert_close(values["ab05_residual_rl"], .311, "Fig. 5 Proposed", 5e-4)
    assert_close(values["ddp_gp"], .584, "Fig. 5 DDP+GP", 5e-4)
    assert_close(100*(1-values["ab05_residual_rl"]/values["ddp_gp"]), 46.8, "Fig. 5 reduction (%)", .05)
    for extension in ("pdf", "png"):
        path = ROOT / "reproduced/figures" / f"figure_5_circular_trajectory.{extension}"
        if not path.is_file() or path.stat().st_size == 0:
            raise AssertionError(f"Missing Fig. 5 output: {path.name}")


def validate_gallery() -> None:
    from matplotlib.image import imread
    from plot_trajectory_examples import (
        EXAMPLES, GALLERY, METHODS, ORIENTATION_COLUMNS, ORIENTATION_MAX_COLUMN,
        read_curves,
    )
    from import_adaptive_summaries import TRAJECTORIES

    curves = read_curves()
    components = curves[list(ORIENTATION_COLUMNS)].to_numpy()
    norms = curves.ee_orientation_error_deg.to_numpy()
    max_components = curves[ORIENTATION_MAX_COLUMN].to_numpy()
    # Norms and maxima are averaged per run; they are not recomputed from
    # the mean vector. Convexity supplies bounds that survive averaging.
    tolerance = 1e-8
    if (np.any(max_components < 0)
            or np.any(np.max(np.abs(components), axis=1) > max_components + tolerance)
            or np.any(np.linalg.norm(components, axis=1) > norms + tolerance)
            or np.any(max_components > norms + tolerance)
            or np.any(norms > np.sqrt(3) * max_components + tolerance)):
        raise AssertionError("Inconsistent orientation-component summaries")
    manifest = json.loads((GALLERY / "adaptive_curve_provenance.json").read_text(encoding="utf-8"))
    expected_keys = {(example[0], method) for example in EXAMPLES for method in METHODS.values()}
    records = manifest["curves"]
    if len(records) != len(expected_keys) or {(r["example"], r["method"]) for r in records} != expected_keys:
        raise AssertionError("Unexpected gallery provenance groups")
    tracking = pd.read_csv(ROOT / "hardware/tracking_and_torque/per_trajectory_metrics.csv")
    payload = pd.read_csv(ROOT / "hardware/payload_robustness/per_trajectory_metrics.csv")
    for record in records:
        subset = curves[(curves.example == record["example"]) & (curves.method == record["method"])]
        if len(subset) != record["samples"] or set(subset.n_runs) != {record["n_runs"]}:
            raise AssertionError("Gallery samples or repeat counts differ from provenance")
        trajectory = TRAJECTORIES[record["trajectory_id"]]
        if record["payload_kg"] == 0:
            source = tracking[(tracking.method == record["method"]) & (tracking.trajectory == trajectory)]
        else:
            source = payload[(payload.method == record["method"]) & (payload.trajectory == trajectory)
                             & (payload.payload_kg == record["payload_kg"])]
        if len(source) != 1:
            raise AssertionError("Ambiguous gallery metric source")
        for source_field, provenance_field in (
            ("ee_position_mean_mm", "full_window_ee_position_mean_mm"),
            ("ee_orientation_mean_deg", "full_window_ee_orientation_mean_deg"),
        ):
            assert_close(source.iloc[0][source_field], record[provenance_field],
                         f"Gallery metric source {record['example']} {record['method']}", 1e-9)
    for example in EXAMPLES:
        expected_size = {
            "3d": (1260, 1440), "xy": (1260, 1260), "xz": (1260, 1260),
            "orientation": (1260, 1800),
        }[example[4]]
        for method in METHODS:
            image = GALLERY / f"{example[0]}_{method}.png"
            if not image.is_file() or image.stat().st_size == 0:
                raise AssertionError(f"Missing gallery image: {image.name}")
            if imread(image).shape[:2] != expected_size:
                raise AssertionError(f"Gallery image dimensions differ from the original: {image.name}")


def validate_portability() -> None:
    for path in ROOT.rglob("*"):
        if path.suffix.lower() in {".bag", ".db3", ".mcap", ".pkl", ".npz", ".zip"}:
            raise AssertionError(f"Raw logs or model archives are not release files: {path}")
        if path.suffix.lower() not in {".csv", ".md", ".py", ".txt", ".json"}:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        has_windows_path = re.search(r"(?<![A-Za-z0-9_])[A-Za-z]:[\\/]", text) is not None
        has_home_path = re.search(r"/(?:home|Users)/[^/\s]+/", text) is not None
        if has_windows_path or has_home_path:
            raise AssertionError(f"Local path in {path}")


def main() -> None:
    validate_source_files()
    validate_table_ii()
    validate_table_iv()
    validate_figure_4_summary()
    validate_table_v()
    validate_circular_trajectory()
    validate_payload()
    validate_gallery()
    validate_portability()
    print("Release validation passed.")


if __name__ == "__main__":
    main()
