"""Export mean gallery curves from the frozen, processed 25-ms batch exports.

This optional maintainer step requires --source-root pointing to the local
batch archive. Public users only need plot_trajectory_examples.py and the
released adaptive_mean_curves.csv. No raw logs are read or copied here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from plot_trajectory_examples import EXAMPLES, GALLERY, METHODS
from gallery_orientation import (
    MODEL_SOURCE, ORIENTATION_COLUMNS, forward_kinematics, orientation_error_deg,
)


SOURCE_COLUMNS = (
    "x_m", "y_m", "z_m", "x_ref_m", "y_ref_m", "z_ref_m",
    "ee_mm", "orientation_deg",
)


def export_curves(source_root: Path, output: Path = GALLERY) -> None:
    batches, source_files = {}, {}
    for grams in sorted({e[3] for e in EXAMPLES}):
        batch_name = f"analysis_20261118_all_{grams}g"
        batch = source_root / batch_name
        series_path, summary_path = batch / "plot_series_25ms.csv.gz", batch / "per_trajectory.csv"
        batches[grams] = (pd.read_csv(series_path), pd.read_csv(summary_path))
        for path in (series_path, summary_path):
            source_files[f"{batch_name}/{path.name}"] = hashlib.sha256(path.read_bytes()).hexdigest()

    outputs, selections = [], []
    verified_samples, max_norm_difference_deg, max_fk_difference_m = 0, 0.0, 0.0
    for example, trajectory, cohort, grams, view in EXAMPLES:
        series, metrics = batches[grams]
        chosen = series[
            series.method.isin(METHODS) & (series.traj == trajectory) & (series.cohort == cohort)
        ]
        if chosen.empty:
            raise ValueError(f"Missing processed curves: {example}")
        times = chosen.groupby(["method", "run"]).time_s.agg(["min", "max"])
        start = int(np.ceil(times["min"].max() / 0.025))
        stop = int(np.floor(min(times["max"].min(), 10.0) / 0.025))
        grid = np.arange(start, stop + 1, dtype=float) * 0.025
        if len(grid) < 350:
            raise ValueError(f"Insufficient shared motion window for {example}")
        for method, label in METHODS.items():
            rows = chosen[chosen.method == method]
            reported = metrics[(metrics.method == method) & (metrics.traj == trajectory)
                               & (metrics.cohort == cohort)]
            if len(reported) != 1:
                raise ValueError(f"Ambiguous summary for {example}, {label}")
            expected_runs = int(reported.iloc[0].n_runs)
            if expected_runs != (3 if grams == 0 else 1) or rows.run.nunique() != expected_runs:
                raise ValueError(f"Curve/summary repeat mismatch for {example}, {label}")
            interpolated, orientation_interpolated = [], []
            for _, run in rows.groupby("run"):
                run = run.sort_values("time_s")
                t = run.time_s.to_numpy()
                if np.any(np.diff(t) <= 0):
                    raise ValueError(f"Non-increasing sample times: {example}, {label}")
                if t[0] > grid[0] or t[-1] < grid[-1]:
                    raise ValueError("Interpolation would extrapolate")
                values = run[list(SOURCE_COLUMNS)].to_numpy()
                if not np.isfinite(values).all():
                    raise ValueError("Non-finite source curves")
                interpolated.append(np.column_stack([
                    np.interp(grid, t, values[:, i]) for i in range(len(SOURCE_COLUMNS))
                ]))
                q = run[[f"q{i}_rad" for i in range(1, 8)]].to_numpy()
                q_ref = run[[f"q_ref{i}_rad" for i in range(1, 8)]].to_numpy()
                actual, reference = forward_kinematics(q), forward_kinematics(q_ref)
                components = orientation_error_deg(actual[:, :3, :3], reference[:, :3, :3])
                norm_difference = float(np.max(np.abs(
                    np.linalg.norm(components, axis=1) - run.orientation_deg.to_numpy()
                )))
                fk_difference = float(max(
                    np.max(np.abs(actual[:, :3, 3] - values[:, :3])),
                    np.max(np.abs(reference[:, :3, 3] - values[:, 3:6])),
                ))
                if norm_difference > 1e-9 or fk_difference > 1e-11:
                    raise ValueError(f"Reconstructed geometry differs from the archive: {example}, {label}")
                verified_samples += len(run)
                max_norm_difference_deg = max(max_norm_difference_deg, norm_difference)
                max_fk_difference_m = max(max_fk_difference_m, fk_difference)
                # Calculate magnitudes before time alignment and repeat averaging.
                derived = np.column_stack([components, np.max(np.abs(components), axis=1)])
                orientation_interpolated.append(np.column_stack([
                    np.interp(grid, t, derived[:, i]) for i in range(len(ORIENTATION_COLUMNS))
                ]))
            mean_curve = np.mean(interpolated, axis=0)
            out = pd.DataFrame(mean_curve, columns=SOURCE_COLUMNS).rename(columns={
                "ee_mm": "ee_position_error_mm", "orientation_deg": "ee_orientation_error_deg",
            })
            out[list(ORIENTATION_COLUMNS)] = np.mean(orientation_interpolated, axis=0)
            out.insert(0, "n_runs", expected_runs)
            out.insert(0, "time_s", grid)
            out.insert(0, "payload_kg", grams / 1000)
            out.insert(0, "trajectory_id", trajectory)
            out.insert(0, "method", label)
            out.insert(0, "set", "Training" if cohort == "regular" else "Held-out")
            out.insert(0, "example", example)
            outputs.append(out)
            selections.append({
                "example": example, "method": label, "payload_kg": grams / 1000,
                "trajectory_id": trajectory, "n_runs": expected_runs, "view": view,
                "samples": len(grid), "first_time_s": float(grid[0]), "last_time_s": float(grid[-1]),
                "full_window_ee_position_mean_mm": float(reported.iloc[0].ee_mean_mm_mean),
                "full_window_ee_orientation_mean_deg": float(reported.iloc[0].ori_mean_deg_mean),
            })
    output.mkdir(parents=True, exist_ok=True)
    pd.concat(outputs, ignore_index=True).to_csv(output / "adaptive_mean_curves.csv", index=False,
                                               float_format="%.10g")
    provenance = {
        "coordinate_frame": "panda_link8",
        "reference": "command_plan",
        "sample_spacing_s": 0.025,
        "aggregation": "Linear time alignment of each processed run followed by an equal-weight mean over runs; no extrapolation.",
        "errors": "Error columns average the per-run error magnitudes, not the error of the averaged Cartesian path.",
        "use": "Qualitative visualization only. Table metrics use original full-motion-window per-run statistics, then repeat and trajectory means.",
        "source_sha256": source_files,
        "curves": selections,
        "orientation_components": {
            "definition": "Rotation vector log(R_reference.T @ R_actual), in degrees, expressed in the reference pose frame; not Euler angles.",
            "aggregation": "Compute signed components and maximum absolute component per source sample, interpolate each run to the shared grid, then average runs equally. The original orientation-error norm column is unchanged.",
            "kinematics": MODEL_SOURCE,
            "verification": {
                "source_samples": verified_samples,
                "maximum_norm_difference_deg": max_norm_difference_deg,
                "norm_tolerance_deg": 1e-9,
                "maximum_fk_position_difference_m": max_fk_difference_m,
                "fk_position_tolerance_m": 1e-11,
            },
        },
    }
    (output / "adaptive_curve_provenance.json").write_text(
        json.dumps(provenance, indent=2, ensure_ascii=True) + "\n", encoding="utf-8"
    )
    print(f"Exported {len(outputs)} mean curves; no raw logs or run identifiers copied.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=GALLERY)
    args = parser.parse_args()
    export_curves(args.source_root, args.output_dir)
