"""Reproduce manuscript Fig. 5 from the released, already aggregated CSVs.

Run this file directly, or import ``reproduce_figure_5(figure_dir=None)``.
The original plotting grid, per-method references, and scalar error averages
are retained. No raw logs, TeX installation, or global backend changes are used.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
from matplotlib import font_manager
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "hardware" / "circular_trajectory"
STYLES = {
    "pure_ddp_no_rl": ("Nominal DDP", "#D72650", "-"),
    "friction": ("Friction DDP", "#008766", "-."),
    "slotine_li": ("Slotine\u2013Li", "#9B4494", ":"),
    "ddp_gp": ("DDP+GP", "#BC4B00", "--"),
    "velocity_control": ("Joint velocity control", "#A88B00", "-."),
    "ab05_residual_rl": ("Proposed", "#005992", "-"),
}
XYZ = ["x_m", "y_m", "z_m"]
REF = ["x_ref_m", "y_ref_m", "z_ref_m"]
LIMITS = np.array([[242., 252.], [-5., 5.], [405., 415.]])


def _load_curves():
    """Read the published means without re-averaging or resampling them."""
    rows = pd.read_csv(DATA_DIR / "plotted_mean_curves.csv", float_precision="round_trip")
    metrics = pd.read_csv(DATA_DIR / "selected_case_metrics.csv", float_precision="round_trip")
    if set(rows["method"]) != set(STYLES) or set(metrics["method"]) != set(STYLES):
        raise ValueError("Expected the six recorded controllers")
    if metrics["method"].duplicated().any():
        raise ValueError("Expected one full-rate metric row per controller")
    numeric = XYZ + REF + ["ee_mm"]
    curves = {}
    grid = None
    for method in STYLES:
        block = rows[rows["method"] == method]
        times = block["time_s"].to_numpy()
        if not np.isfinite(block[["time_s"] + numeric].to_numpy()).all():
            raise ValueError(f"Nonfinite plot data for {method}")
        if len(times) != 398 or not np.all(np.diff(times) > 0):
            raise ValueError(f"Unexpected plot grid for {method}")
        if grid is None:
            grid = times
        elif not np.array_equal(grid, times):
            raise ValueError("Controllers must share the archived plotting grid")
        expected = int(metrics.loc[metrics["method"] == method, "n_runs"].iloc[0])
        if not (block["n_runs"] == expected).all():
            raise ValueError(f"Inconsistent repeat count for {method}")
        curves[method] = {column: block[column].to_numpy() for column in numeric}
    if not np.allclose(grid, np.arange(1, 399) * .025, rtol=0, atol=2e-14):
        raise ValueError("Unexpected common 25-ms plotting window")
    return grid, curves


def distance_to_path(points, path):
    """Maximum point-to-polyline distance, as in the manuscript renderer."""
    starts, segments = path[:-1], np.diff(path, axis=0)
    offsets = points[:, None, :] - starts[None, :, :]
    fractions = np.clip(np.sum(offsets * segments[None, :, :], axis=2)
                        / np.maximum(np.sum(segments ** 2, axis=1), 1e-24), 0, 1)
    return float(np.linalg.norm(offsets - fractions[:, :, None] * segments[None, :, :],
                                axis=2).min(axis=1).max())


def clip_path(points, limits):
    """Intersect straight plotted segments with the original physical box."""
    output = []
    for p0, p1 in zip(points[:-1], points[1:]):
        delta = p1 - p0
        lo, hi = 0., 1.
        for j, (lower, upper) in enumerate(limits):
            if abs(delta[j]) < 1e-12:
                if not lower <= p0[j] <= upper:
                    lo, hi = 1., 0.
                    break
            else:
                a, b = sorted(((lower - p0[j]) / delta[j], (upper - p0[j]) / delta[j]))
                lo, hi = max(lo, a), min(hi, b)
        if lo <= hi:
            start, end = p0 + lo * delta, p0 + hi * delta
            if output and not np.allclose(output[-1], start, atol=1e-10, rtol=0):
                output.append(np.full(3, np.nan))
            output.extend([start, end])
    if not output:
        raise ValueError("No path segment intersects the common detail window")
    return np.array(output)


def _serif_font():
    try:
        font_manager.findfont("Times New Roman", fallback_to_default=False)
        return "Times New Roman"
    except ValueError:
        return "STIXGeneral"


def render_plot(grid, curves, outputs):
    """Preserve the reviewed manuscript layout with a portable local canvas."""
    reference = np.column_stack([curves["ab05_residual_rl"][col] for col in REF]) * 1000
    geometry = {}
    for method, curve in curves.items():
        ref = np.column_stack([curve[col] for col in REF]) * 1000
        geometry[method] = max(distance_to_path(ref, reference), distance_to_path(reference, ref))
    if max(geometry.values()) > .1:
        raise ValueError("Recorded reference paths are too different for one spatial reference line")
    style = {
        "font.family": "serif", "font.serif": [_serif_font(), "STIXGeneral"],
        "mathtext.fontset": "stix", "text.usetex": False,
        "font.size": 9, "axes.linewidth": .55, "axes.labelsize": 9,
        "xtick.labelsize": 8, "ytick.labelsize": 8, "pdf.fonttype": 42,
        "savefig.bbox": None,
    }
    with matplotlib.rc_context(style):
        fig = Figure(figsize=(7.2, 6.0))
        FigureCanvasAgg(fig)
        handles = [Line2D([], [], color="black", linestyle=(0, (3.5, 2.5)), linewidth=1., label="reference")]
        handles += [Line2D([], [], color=color, linestyle=linestyle, linewidth=1.4, label=label)
                    for label, color, linestyle in STYLES.values()]
        fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.51, .995), ncol=4,
                   frameon=False, fontsize=8.5, handlelength=2.6, columnspacing=1.5, labelspacing=.65)

        # Equal physical panel sizes: 0.37 * 7.2 in = 0.444 * 6.0 in.
        overview = fig.add_axes([.085, .43, .37, .444])
        detail = fig.add_axes([.555, .43, .37, .444], projection="3d")
        error = fig.add_axes([.10, .09, .85, .20])
        overview.plot(reference[:, 0], reference[:, 2], color="black", linewidth=.9,
                      linestyle=(0, (3.5, 2.5)), zorder=10)
        ref_detail = clip_path(reference, LIMITS)
        detail.plot(*ref_detail.T, color="black", linewidth=1., linestyle=(0, (3.5, 2.5)), zorder=10)
        for method, (_, color, linestyle) in STYLES.items():
            curve = curves[method]
            actual = np.column_stack([curve[col] for col in XYZ]) * 1000
            local = clip_path(actual, LIMITS)
            width = 1.65 if method == "ab05_residual_rl" else 1.35
            order = 8 if method == "ab05_residual_rl" else 4
            overview.plot(actual[:, 0], actual[:, 2], color=color, linestyle=linestyle,
                          linewidth=.95 if method == "ab05_residual_rl" else .75, zorder=order)
            detail.plot(*local.T, color=color, linestyle=linestyle, linewidth=width, zorder=order)
            error.plot(grid, curve["ee_mm"], color=color, linestyle=linestyle,
                       linewidth=width-.25, zorder=order)

        overview.add_patch(Rectangle((LIMITS[0, 0], LIMITS[2, 0]), 10, 10,
                                    fill=False, edgecolor="black", linewidth=.9, zorder=11))
        overview.annotate("detail", xy=(247, 410), xytext=(333, 450), fontsize=8,
                          ha="center", arrowprops=dict(arrowstyle="->", color=".35", linewidth=.7))
        overview.set(xlim=(195, 420), ylim=(380, 605), aspect="equal",
                     xlabel=r"$x$ (mm)", ylabel=r"$z$ (mm)")
        overview.set_xticks([220, 300, 380])
        overview.set_yticks([400, 480, 560])
        fig.text(.27, .89, "(a) EE path", fontsize=9, ha="center", va="bottom")

        detail.view_init(elev=23, azim=-62)
        detail.set_xlim(*LIMITS[0])
        detail.set_ylim(*LIMITS[1])
        detail.set_zlim(*LIMITS[2])
        detail.set_box_aspect([1, 1, 1], zoom=.99)
        detail.set_xticks([242, 247, 252])
        detail.set_yticks([-4, 0, 4])
        detail.set_zticks([406, 410, 414])
        detail.set_xlabel(r"$x$ (mm)", labelpad=0)
        detail.set_ylabel(r"$y$ (mm)", labelpad=0)
        detail.set_zlabel(r"$z$ (mm)", labelpad=0)
        detail.tick_params(pad=0, labelsize=8)
        fig.text(.74, .89, "(b) Enlarged view", fontsize=9, ha="center", va="bottom")
        detail.grid(False)
        for axis in (detail.xaxis, detail.yaxis, detail.zaxis):
            axis.pane.fill = False
            axis.pane.set_edgecolor(".85")
            axis.line.set_color(".65")

        error.set(xlim=(0, 10), ylim=(0, 4.4), xlabel="Time (s)", ylabel="EE position error (mm)")
        error.set_yticks([0, 1, 2, 3, 4])
        error.set_title("(c) EE position error", fontsize=9, pad=8)
        for axis in (overview, error):
            axis.spines[["top", "right"]].set_visible(False)
            axis.tick_params(length=2.5, width=.55)
            axis.set_axisbelow(True)
            axis.grid(False, which="both", axis="both")
        for output in outputs:
            fig.savefig(output, dpi=300, metadata={"Title": "EE position tracking on a circular trajectory"})
        fig.clear()
    return geometry


def reproduce_figure_5(figure_dir=None) -> tuple[Path, Path]:
    """Write the PDF and PNG and return their paths; default to released figures."""
    output_dir = Path(figure_dir) if figure_dir is not None else ROOT / "reproduced" / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = tuple(output_dir / f"figure_5_circular_trajectory.{suffix}" for suffix in ("pdf", "png"))
    grid, curves = _load_curves()
    render_plot(grid, curves, outputs)
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--figure-dir", type=Path, help="Output directory for the PDF and PNG")
    args = parser.parse_args()
    for output in reproduce_figure_5(args.figure_dir):
        print(output)


if __name__ == "__main__":
    main()
