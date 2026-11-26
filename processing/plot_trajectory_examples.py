"""Rebuild the two adaptive-controller gallery images from released mean curves.

Run from any working directory: python processing/plot_trajectory_examples.py
No robot logs, models, or external experiment folders are needed.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
GALLERY = ROOT / "hardware" / "trajectory_examples"
METHODS = {"slotine_li": "Slotine-Li", "ddp_gp": "DDP+GP"}
REFERENCE_COLOR = "tab:orange"
ROBOT_COLOR = "tab:green"
ORIENTATION_COLUMNS = tuple(f"ee_orientation_r{axis}_deg" for axis in "xyz")
ORIENTATION_MAX_COLUMN = "ee_orientation_max_abs_component_deg"
# example, source trajectory, cohort, added mass (g), view
EXAMPLES = (
    ("training_joint_sine", "panda_ddp_joint_sine", "regular", 0, "3d"),
    ("large_figure8_xz", "panda_ddp_figure8_xz_large", "regular", 0, "xz"),
    ("orientation_swing", "panda_ddp_orientation_swing", "regular", 0, "orientation"),
    ("heldout_lissajous", "panda_ddp_test_lissajous_xy", "test", 0, "3d"),
    ("joint_sine_200g", "panda_ddp_joint_sine", "regular", 200, "3d"),
    ("joint_sine_1kg", "panda_ddp_joint_sine", "regular", 1000, "3d"),
    ("circle_xy_1kg", "panda_ddp_circle_xy", "regular", 1000, "xy"),
)
CURVE_COLUMNS = (
    "x_m", "y_m", "z_m", "x_ref_m", "y_ref_m", "z_ref_m",
    "ee_position_error_mm", "ee_orientation_error_deg",
)


def read_curves(path: Path = GALLERY / "adaptive_mean_curves.csv") -> pd.DataFrame:
    data = pd.read_csv(path)
    required = set(CURVE_COLUMNS) | set(ORIENTATION_COLUMNS) | {ORIENTATION_MAX_COLUMN}
    if missing := required - set(data.columns):
        raise ValueError(f"Missing gallery curve fields: {sorted(missing)}")
    keys = ["example", "method", "time_s"]
    if data.duplicated(keys).any():
        raise ValueError("Duplicate gallery curve samples")
    numeric = data.select_dtypes(include=[np.number])
    if not np.isfinite(numeric.to_numpy()).all():
        raise ValueError("Non-finite gallery curve values")
    expected_groups = {(e[0], label) for e in EXAMPLES for label in METHODS.values()}
    if set(data.groupby(["example", "method"]).groups) != expected_groups:
        raise ValueError("Missing or unexpected gallery curve groups")
    for example, trajectory, cohort, grams, _ in EXAMPLES:
        block = data[data.example == example]
        expected_set = "Training" if cohort == "regular" else "Held-out"
        if set(block["set"]) != {expected_set} or set(block.payload_kg) != {grams / 1000}:
            raise ValueError(f"Incorrect condition for {example}")
        if set(block.trajectory_id) != {trajectory}:
            raise ValueError(f"Incorrect trajectory for {example}")
        if set(block.n_runs) != {3 if grams == 0 else 1}:
            raise ValueError(f"Incorrect repeat counts for {example}")
        grids = []
        for _, curve in block.groupby("method"):
            t = curve.sort_values("time_s").time_s.to_numpy()
            if len(t) < 350 or t[0] < 0 or t[-1] > 10:
                raise ValueError(f"Invalid motion window for {example}")
            if not np.allclose(np.diff(t), 0.025, rtol=0, atol=1e-10):
                raise ValueError(f"Invalid 25-ms grid for {example}")
            grids.append(t)
        if not np.array_equal(grids[0], grids[1]):
            raise ValueError(f"Methods use different times for {example}")
    return data


def spatial_points(block: pd.DataFrame, coordinates: str) -> np.ndarray:
    """Use both methods to set common axes without changing either curve."""
    return np.vstack([
        block[[f"{c}{suffix}_m" for c in coordinates]].to_numpy()
        for suffix in ("", "_ref")
    ]) * 1000


def plot_gallery(data: pd.DataFrame, output: Path = GALLERY) -> None:
    # The original gallery uses Matplotlib defaults: DejaVu Sans, boxed
    # legends, and the standard 3-D panes and camera.
    plt.rcdefaults()
    output.mkdir(parents=True, exist_ok=True)
    for example, _, _, _, view in EXAMPLES:
        block = data[data.example == example]
        for slug, label in METHODS.items():
            curve = block[block.method == label].sort_values("time_s")
            if view == "3d":
                fig = plt.figure(figsize=(8, 7))
                ax = fig.add_subplot(111, projection="3d")
                ax.plot(*(curve[f"{c}_ref_m"] * 1000 for c in "xyz"),
                        color=REFERENCE_COLOR, ls="--", lw=1.2,
                        label="command_plan reference 3d")
                ax.plot(*(curve[f"{c}_m"] * 1000 for c in "xyz"),
                        color=ROBOT_COLOR, lw=1.4, label="robot 3d")
                ax.auto_scale_xyz(*spatial_points(block, "xyz").T)
                ax.set(xlabel="x [mm]", ylabel="y [mm]", zlabel="z [mm]",
                       title="End-Effector 3D Comparison (panda_link8)")
                ax.set_box_aspect((4, 4, 3))
                ax.view_init(elev=30, azim=-60)
                ax.grid(True)
                ax.legend()
            elif view in ("xy", "xz"):
                fig, ax = plt.subplots(figsize=(7, 7))
                x, y = view
                ax.plot(curve[f"{x}_ref_m"] * 1000, curve[f"{y}_ref_m"] * 1000,
                        color=REFERENCE_COLOR, ls="--", lw=1.2,
                        label=f"command_plan reference {view}")
                ax.plot(curve[f"{x}_m"] * 1000, curve[f"{y}_m"] * 1000,
                        color=ROBOT_COLOR, lw=1.4, label=f"robot {view}")
                ax.update_datalim(spatial_points(block, view))
                ax.autoscale_view()
                ax.set(xlabel=f"{x} [mm]", ylabel=f"{y} [mm]",
                       title=f"End-Effector {view.upper()} Comparison (panda_link8)")
                ax.axis("equal")
                ax.grid(True, alpha=0.3)
                ax.legend()
            else:
                fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
                for axis, column in zip("xyz", ORIENTATION_COLUMNS):
                    axes[0].plot(curve.time_s, curve[column], lw=1.2,
                                 label=f"r{axis} error [deg]")
                axes[0].axhline(0, color="k", lw=0.8, ls="--", alpha=0.5)
                axes[0].set_title("End-Effector Orientation Error Components vs "
                                  "command_plan reference (panda_link8)")
                axes[1].plot(curve.time_s, curve.ee_orientation_error_deg, lw=1.4,
                             label="orientation error norm")
                axes[1].plot(curve.time_s, curve[ORIENTATION_MAX_COLUMN], lw=1.2,
                             ls="--", label="max abs component")
                axes[1].set_title("End-Effector Orientation Error Norm vs "
                                  "command_plan reference (panda_link8)")
                axes[1].set_xlabel("time [s]")
                for ax, columns in zip(axes, (
                    ORIENTATION_COLUMNS,
                    ("ee_orientation_error_deg", ORIENTATION_MAX_COLUMN),
                )):
                    values = block[list(columns)].to_numpy()
                    ax.update_datalim([(block.time_s.min(), values.min()),
                                      (block.time_s.max(), values.max())])
                    ax.autoscale_view()
                    ax.set_ylabel("error [deg]")
                    ax.grid(True, alpha=0.3)
                    ax.legend()
            fig.tight_layout()
            fig.savefig(output / f"{example}_{slug}.png", dpi=180, facecolor="white")
            plt.close(fig)


if __name__ == "__main__":
    plot_gallery(read_curves())
    print("Reproduced 14 adaptive-controller trajectory images.")
