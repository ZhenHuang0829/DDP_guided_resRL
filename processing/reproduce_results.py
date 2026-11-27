"""Reproduce the numerical tables and data figures reported in the paper."""

from __future__ import annotations

from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
TABLE_DIR = ROOT / "reproduced" / "tables"
FIGURE_DIR = ROOT / "reproduced" / "figures"
HARDWARE_METHODS = [
    "Nominal DDP", "Friction DDP", "Slotine-Li", "DDP+GP",
    "Joint velocity control", "Proposed",
]
TRACKING_METRICS = [
    "joint_position_rms_deg", "joint_velocity_rms_deg_s",
    "ee_position_mean_mm", "ee_orientation_mean_deg",
    "measured_torque_rms_nm", "measured_torque_rate_rms_nm_s",
]


def mean_and_sample_std(
    data: pd.DataFrame,
    group_columns: list[str],
    metrics: list[str],
    order: list[str],
) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    leading_groups = group_columns[:-1]
    grouped = (
        data.groupby(leading_groups, sort=False, dropna=False)
        if leading_groups
        else [((), data)]
    )
    for leading_values, block in grouped:
        if not isinstance(leading_values, tuple):
            leading_values = (leading_values,)
        leading = dict(zip(leading_groups, leading_values))
        for method in order:
            subset = block[block[group_columns[-1]] == method]
            row: dict[str, object] = {
                **leading,
                group_columns[-1]: method,
            }
            for metric in metrics:
                row[f"{metric}_mean"] = subset[metric].mean()
                row[f"{metric}_std"] = subset[metric].std(ddof=1)
            rows.append(row)
    return pd.DataFrame(rows)


def reproduce_table_ii() -> pd.DataFrame:
    data = pd.read_csv(
        ROOT / "simulation/controller_comparison/evaluation_metrics.csv"
    )
    metrics = [
        "mean_step_reward",
        "joint_position_rms_deg",
        "ee_position_rms_mm",
        "ee_orientation_rms_deg",
    ]
    result = mean_and_sample_std(
        data,
        ["method"],
        metrics,
        ["Nominal DDP", "Direct RL", "Proposed"],
    )
    result.to_csv(
        TABLE_DIR / "table_ii_simulation_controller_comparison.csv", index=False
    )
    return result


def reproduce_table_iv() -> pd.DataFrame:
    data = pd.read_csv(
        ROOT / "simulation/component_ablation/evaluation_metrics.csv"
    )
    metrics = [
        "mean_step_reward",
        "joint_position_rms_deg",
        "joint_velocity_rms_deg_s",
        "ee_position_rms_mm",
        "ee_orientation_rms_deg",
        "residual_torque_rms_nm",
        "residual_torque_rate_rms_nm_s",
    ]
    result = mean_and_sample_std(
        data,
        ["method"],
        metrics,
        ["MLP baseline", "MLP + W.", "Rec. + W.", "Rec. + W. + ID", "Proposed"],
    )
    result.to_csv(
        TABLE_DIR / "table_iv_simulation_component_ablation.csv", index=False
    )
    return result


def reproduce_tracking_summary() -> pd.DataFrame:
    data = pd.read_csv(
        ROOT / "hardware/tracking_and_torque/per_trajectory_metrics.csv"
    )
    result = mean_and_sample_std(
        data,
        ["set", "method"],
        TRACKING_METRICS,
        HARDWARE_METHODS,
    )
    result.to_csv(
        TABLE_DIR / "figure_5_tracking_torque_summary.csv", index=False
    )
    return result


def reproduce_table_v() -> pd.DataFrame:
    data = pd.read_csv(ROOT / "hardware/dual_endpoint/per_trajectory_metrics.csv")
    metrics = [
        "ee_position_mean_mm",
        "joint_position_rms_deg",
        "joint_velocity_rms_deg_s",
        "commanded_torque_rate_rms_nm_s",
        "measured_torque_rate_rms_nm_s",
        "measured_torque_second_derivative_rms_nm_s2",
    ]
    result = mean_and_sample_std(
        data,
        ["set", "method"],
        metrics,
        ["ZOH residual", "Proposed"],
    )
    result.to_csv(
        TABLE_DIR / "table_v_dual_endpoint_ablation.csv", index=False
    )
    return result


def reproduce_figure_2() -> None:
    data = pd.read_csv(ROOT / "simulation/learning_efficiency/learning_curves.csv")
    def curve(method):
        rows = data[data["method"] == method].sort_values("training_transitions")
        return rows["training_transitions"].to_numpy()/1e6, rows["mean_evaluation_return"].to_numpy()
    d_steps, d_return = curve("Direct RL")
    p_steps, p_return = curve("Proposed")
    figure, axis = plt.subplots(figsize=(4.8, 3.65))
    figure.subplots_adjust(left=.14, right=.98, bottom=.14, top=.89)
    common, d_idx, p_idx = np.intersect1d(d_steps, p_steps, return_indices=True)
    d_peak, p_peak = np.argmax(d_return), np.argmax(p_return)
    shade = dict(color="#0072B2", alpha=.10, linewidth=0, zorder=0)
    axis.fill_between(common, d_return[d_idx], p_return[p_idx], **shade)
    axis.fill_between([p_steps[p_peak], d_steps[d_peak]], d_return[d_peak], p_return[p_peak], **shade)
    # Match the manuscript's continuation boundaries and display-only sampling.
    # All evaluations remain in the CSV and in the numerical comparisons.
    bounds = (0, 19.9, 29.4, 38.5, 49.7, 60.0, 79.2, 100.)
    for index, (start, end) in enumerate(zip(bounds[:-1], bounds[1:])):
        selected = (d_steps > start) & (d_steps <= end)
        x, y = d_steps[selected], d_return[selected]
        show = (x <= 10) | np.isclose(x, np.round(x))
        show[[0, -1]] = True
        axis.plot(x[show], y[show], color="#D55E00", dashes=(4, 1.6), lw=1.3,
                  label="Direct RL" if index == 0 else None)
    overlap = p_steps >= d_steps[0]
    axis.plot(p_steps[overlap], p_return[overlap], color="#0072B2", lw=1.65, label="Proposed")
    comparisons = (
        (common[-1], 1.14*common[-1], d_return[d_idx[-1]], p_return[p_idx[-1]], f"at {common[-1]:g}M", "left"),
        (82, 37, d_return[d_peak], p_return[p_peak], "peak returns", "center"),
    )
    for arrow_x, label_x, direct, proposed, label, align in comparisons:
        axis.annotate("", xy=(arrow_x, proposed), xytext=(arrow_x, direct),
                      arrowprops=dict(arrowstyle="<->", color="#333333", linewidth=.85))
        axis.text(label_x, (direct+proposed)/2, f"{100*(proposed/direct-1):+.1f}%\n{label}",
                  ha=align, va="center", color="#222222", fontsize=8)
    axis.set_xscale("log")
    axis.set_xlim(.55, 110.0)
    axis.set_xticks([.6, 1., 10., 100.], labels=["0.6", "1", "10", "100"])
    axis.set_ylim(0.0, 3700.0)
    axis.set_yticks([0, 1000, 2000, 3000])
    axis.set_xlabel("Training transitions (million, log scale)")
    axis.set_ylabel("Mean evaluation return")
    axis.grid(False, which="both", axis="both")
    axis.set_axisbelow(True)
    axis.legend(loc="lower center", bbox_to_anchor=(.5, 1.015), ncol=2,
                frameon=True, borderpad=.3, borderaxespad=0)
    figure.savefig(FIGURE_DIR / "figure_2_learning_efficiency.png", dpi=220)
    figure.savefig(FIGURE_DIR / "figure_2_learning_efficiency.pdf")
    plt.close(figure)


def reproduce_figure_3() -> None:
    data = pd.read_csv(ROOT / "simulation/component_ablation/learning_curves.csv")
    styles = [
        ("MLP baseline", "#707070", (0, (3, 1))),
        ("MLP + W.", "#B57300", (0, (5, 1))),
        ("Rec. + W.", "#4699C1", (0, (5, 1, 1, 1))),
        ("Rec. + W. + ID", "#008C68", (0, (1, 1))),
        ("Proposed", "#0072B2", "-"),
    ]
    figure, axis = plt.subplots(figsize=(4.8, 2.65))
    figure.subplots_adjust(left=.15, right=.98, bottom=.19, top=.96)
    for method, color, linestyle in styles:
        subset = data[data["method"] == method].sort_values("training_transitions")
        axis.plot(
            subset["training_transitions"] / 1.0e6,
            subset["mean_evaluation_return"],
            label=method,
            color=color,
            linestyle=linestyle,
            linewidth=1.6 if method == "Proposed" else 1.15,
        )
        axis.plot(subset["training_transitions"].iloc[-1]/1e6, subset["mean_evaluation_return"].iloc[-1],
                  marker="o", markersize=2.6, color=color, clip_on=False)
    axis.set_xlim(0.0, 10.0)
    axis.set_ylim(1800.0, 3500.0)
    axis.set_yticks([2000, 2500, 3000, 3500])
    axis.set_xlabel("Training transitions (million)")
    axis.set_ylabel("Mean evaluation return")
    axis.grid(False, which="both", axis="both")
    axis.set_axisbelow(True)
    axis.legend(loc="lower right", frameon=False, labelspacing=.25)
    figure.savefig(FIGURE_DIR / "figure_3_component_ablation.png", dpi=220)
    figure.savefig(FIGURE_DIR / "figure_3_component_ablation.pdf")
    plt.close(figure)


def reproduce_figure_5(summary: pd.DataFrame) -> None:
    """Match the manuscript's six-metric figure and bold the within-set minima."""
    panels = [
        ("joint_position_rms_deg", "Joint position RMSE (deg)", (0, .5, 1, 1.5, 2)),
        ("joint_velocity_rms_deg_s", "Joint velocity RMSE (deg/s)", (0, 1, 2, 3, 4)),
        ("ee_position_mean_mm", "Mean EE position error (mm)", (0, 1, 2, 3, 4)),
        ("ee_orientation_mean_deg", "Mean EE orientation error (deg)", (0, .5, 1, 1.5, 2)),
        ("measured_torque_rms_nm", "Measured torque RMS (Nm)", (0, 2, 4, 6)),
        ("measured_torque_rate_rms_nm_s", "Measured torque-rate RMS (Nm/s)", (0, 20, 40, 60)),
    ]
    figure, axes = plt.subplots(3, 2, figsize=(6.5, 6.3), sharex=True)
    figure.subplots_adjust(left=.063, right=.99, top=.925, bottom=.083, hspace=.70, wspace=.18)
    x = np.arange(len(HARDWARE_METHODS))
    labels = ["Nominal\nDDP", "Friction\nDDP", "SL", "DDP+GP", "Joint\nvelocity\ncontrol", "Proposed"]
    sets = ("Training", "Held-out")
    styles = (dict(facecolor="#306384", edgecolor="#306384"),
              dict(facecolor="#f8f1df", edgecolor="#785725"))
    for axis, (metric, title, ticks) in zip(axes.flat, panels):
        for set_name, offset, style in zip(sets, (-.19, .19), styles):
            subset = summary[summary["set"] == set_name].set_index("method").loc[HARDWARE_METHODS]
            # Manuscript bars use the accompanying CSV rounded to three decimals.
            values = subset[metric + "_mean"].round(3).to_numpy()
            if not np.isfinite(values).all() or (values < 0).any() or (values > ticks[-1]).any():
                raise ValueError(f"Invalid plotted Fig. 5 values: {metric}")
            bars = axis.bar(x + offset, values, width=.34, linewidth=.6, label=set_name, **style)
            labels_at_bars = axis.bar_label(bars, fmt="%.3f", rotation=90, fontsize=7.5, padding=2)
            for annotation, value in zip(labels_at_bars, values):
                if value == values.min():
                    annotation.set_fontweight("bold")
        axis.set_title(title, fontsize=9, pad=4)
        axis.set_ylim(0, ticks[-1])
        axis.set_yticks(ticks, labels=[f"{tick:g}" for tick in ticks])
        axis.set_xticks(x, labels=labels)
        axis.set_xlim(-.52, len(x)-.48)
        axis.tick_params(axis="x", labelbottom=True, labelsize=8.1, pad=3.2, length=0)
        axis.tick_params(axis="y", labelsize=8, pad=2.5, length=2.3, width=.55)
        for spine in axis.spines.values():
            spine.set_linewidth(.55)
        axis.spines[["top", "right"]].set_visible(False)
        axis.grid(False)
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, legend_labels, loc="upper center", ncol=2, frameon=False, fontsize=8.5,
                  borderaxespad=.15, handlelength=1.6, columnspacing=1.6)
    figure.savefig(FIGURE_DIR / "figure_5_hardware_tracking_and_torque.png", dpi=220)
    figure.savefig(FIGURE_DIR / "figure_5_hardware_tracking_and_torque.pdf")
    plt.close(figure)


def reproduce_figure_7() -> pd.DataFrame:
    data = pd.read_csv(
        ROOT / "hardware/payload_robustness/per_trajectory_metrics.csv"
    )
    metrics = [
        "ee_position_mean_mm",
        "ee_orientation_mean_deg",
        "joint_position_rms_deg",
    ]
    summary = (
        data.groupby(["method", "payload_kg"], as_index=False)[metrics]
        .mean()
        .sort_values(["method", "payload_kg"])
    )
    summary.to_csv(TABLE_DIR / "figure_7_payload_summary.csv", index=False)

    methods = [
        ("Nominal DDP", "#555555", "s", "--"),
        ("Friction DDP", "#008766", "^", "-."),
        ("Slotine-Li", "#9B4494", "D", ":"),
        ("DDP+GP", "#BC4B00", "v", "--"),
        ("Proposed", "#005992", "o", "-"),
    ]
    panels = [
        ("ee_position_mean_mm", "Mean EE position error (mm)", 8.25, (0, 2, 4, 6, 8)),
        ("ee_orientation_mean_deg", "Mean EE orientation error (deg)", 1.25, (0, .3, .6, .9, 1.2)),
        ("joint_position_rms_deg", "Joint position RMSE (deg)", 1.6, (0, .4, .8, 1.2, 1.6)),
    ]
    figure, axes = plt.subplots(1, 3, figsize=(6.5, 2.5), sharex=True)
    for axis, (metric, ylabel, upper, ticks) in zip(axes, panels):
        axis.axvspan(0.8, 1.045, color="#F4EBDD", zorder=0)
        for method, color, marker, linestyle in methods:
            subset = summary[summary["method"] == method]
            axis.plot(
                subset["payload_kg"],
                subset[metric],
                label=method.replace("Slotine-Li", "Slotine–Li"),
                color=color,
                marker=marker,
                linestyle=linestyle,
                markersize=4.2, markeredgewidth=.9,
                markerfacecolor=color if method == "Proposed" else "white",
                linewidth=1.4,
            )
        axis.set_xlabel("Added mass (kg)")
        axis.set_ylabel(ylabel)
        axis.set_xticks([0, .2, .6, 1.0], labels=["0", "0.2", "0.6", "1.0"])
        axis.set_xlim(-.045, 1.045)
        axis.set_ylim(0, upper)
        axis.set_yticks(ticks)
        axis.spines[["top", "right"]].set_visible(False)
        axis.grid(False)
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", ncol=5, frameon=False, fontsize=9,
                  handlelength=2.1, columnspacing=1.25)
    figure.subplots_adjust(left=.072, right=.988, bottom=.21, top=.81, wspace=.43)
    figure.savefig(FIGURE_DIR / "figure_7_payload_robustness.png", dpi=220)
    figure.savefig(FIGURE_DIR / "figure_7_payload_robustness.pdf")
    plt.close(figure)
    return summary


def main() -> None:
    plt.rcParams.update({
        "font.family": "serif", "font.serif": ["Times New Roman", "STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix", "font.size": 9, "axes.linewidth": .65,
        "xtick.labelsize": 8.5, "ytick.labelsize": 8.5, "legend.fontsize": 8,
        "pdf.fonttype": 42,
    })
    TABLE_DIR.mkdir(parents=True, exist_ok=True)
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    reproduce_table_ii()
    reproduce_table_iv()
    hardware_summary = reproduce_tracking_summary()
    reproduce_table_v()
    reproduce_figure_2()
    reproduce_figure_3()
    reproduce_figure_5(hardware_summary)
    from plot_circular_trajectory import reproduce_figure_6
    reproduce_figure_6(FIGURE_DIR)
    reproduce_figure_7()
    print(f"Reproduced tables in {TABLE_DIR}")
    print(f"Reproduced figures in {FIGURE_DIR}")


if __name__ == "__main__":
    main()
