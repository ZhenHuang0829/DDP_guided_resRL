# Real-robot trajectory examples

The gallery shows measured end-effector trajectories against the commanded
reference for the hardware comparisons in Figs. 5 and 7. Spatial views,
planar projections, and orientation-error traces follow the original gallery.

The existing controller images are retained from the original release. The
new Slotine-Li and DDP+GP images use the processed 25-ms curve exports from the
same experiments as their per-trajectory metrics. The added zero-payload
examples average all three repetitions after time alignment; loaded examples
have one run per method and trajectory. The two added methods share coordinate
limits and camera angles within each example. Their reference is the recorded
command plan, evaluated in the `panda_link8` frame.
The added images retain the original gallery's fonts, dimensions, camera,
grid, and orange dashed reference and green measured-trajectory curves.
They omit the separate plan-file curve because that signal is not included
in the processed exports.

As reported in the manuscript, the Fig. 5 comparison uses no added payload.
The images are matched examples from separately recorded controller trials.

These are qualitative illustrations. The reported summary values use full-motion-window
per-run metrics, averaged within each trajectory and then equally across
trajectories. They must not be recomputed from the time-aligned mean curves:
averaging Cartesian coordinates can reduce visible tracking error, and the
25-ms representation does not preserve every high-rate fluctuation. The
released error curves average per-run error magnitudes, not the error of the
mean Cartesian path. Orientation panels show the three rotation-vector
components above and the rotation angle and maximum absolute component below.
Components use `log(R_ref.T @ R_measured)` in the reference frame, computed
from each run's recorded joint positions before time alignment and averaging.
The lower panel averages per-run magnitudes, rather than taking the norm or
maximum of the averaged components.

## Training trajectory: joint-space sinusoidal motion

These examples correspond to `Joint-space sinusoidal motion` with `set = Training` in
[`../tracking_and_torque/per_trajectory_metrics.csv`](../tracking_and_torque/per_trajectory_metrics.csv).

| Nominal DDP | Friction DDP | Joint velocity control | Slotine-Li | DDP+GP | Proposed |
|---|---|---|---|---|---|
| ![Nominal DDP: Joint-space sinusoidal motion](training_joint_sine_nominal_ddp.png) | ![Friction DDP: Joint-space sinusoidal motion](training_joint_sine_friction_ddp.png) | ![Joint velocity control: Joint-space sinusoidal motion](training_joint_sine_joint_velocity_control.png) | ![Slotine-Li: Joint-space sinusoidal motion](training_joint_sine_slotine_li.png) | ![DDP+GP: Joint-space sinusoidal motion](training_joint_sine_ddp_gp.png) | ![Proposed: Joint-space sinusoidal motion](training_joint_sine_proposed.png) |

## Training trajectory: large figure-eight in the XZ plane

These examples correspond to `Large figure-eight in the XZ plane` with `set = Training` in
[`../tracking_and_torque/per_trajectory_metrics.csv`](../tracking_and_torque/per_trajectory_metrics.csv).

| Nominal DDP | Friction DDP | Joint velocity control | Slotine-Li | DDP+GP | Proposed |
|---|---|---|---|---|---|
| ![Nominal DDP: Large figure-eight in the XZ plane](large_figure8_xz_nominal_ddp.png) | ![Friction DDP: Large figure-eight in the XZ plane](large_figure8_xz_friction_ddp.png) | ![Joint velocity control: Large figure-eight in the XZ plane](large_figure8_xz_joint_velocity_control.png) | ![Slotine-Li: Large figure-eight in the XZ plane](large_figure8_xz_slotine_li.png) | ![DDP+GP: Large figure-eight in the XZ plane](large_figure8_xz_ddp_gp.png) | ![Proposed: Large figure-eight in the XZ plane](large_figure8_xz_proposed.png) |

## Training orientation task: orientation swing

These examples correspond to `Orientation swing` with `set = Training` in
[`../tracking_and_torque/per_trajectory_metrics.csv`](../tracking_and_torque/per_trajectory_metrics.csv).

| Nominal DDP | Friction DDP | Joint velocity control | Slotine-Li | DDP+GP | Proposed |
|---|---|---|---|---|---|
| ![Nominal DDP: Orientation swing](orientation_swing_nominal_ddp.png) | ![Friction DDP: Orientation swing](orientation_swing_friction_ddp.png) | ![Joint velocity control: Orientation swing](orientation_swing_joint_velocity_control.png) | ![Slotine-Li: Orientation swing](orientation_swing_slotine_li.png) | ![DDP+GP: Orientation swing](orientation_swing_ddp_gp.png) | ![Proposed: Orientation swing](orientation_swing_proposed.png) |

## Held-out trajectory: horizontal Lissajous motion

These examples correspond to `Horizontal Lissajous motion` with `set = Held-out` in
[`../tracking_and_torque/per_trajectory_metrics.csv`](../tracking_and_torque/per_trajectory_metrics.csv).

| Nominal DDP | Friction DDP | Joint velocity control | Slotine-Li | DDP+GP | Proposed |
|---|---|---|---|---|---|
| ![Nominal DDP: Horizontal Lissajous motion](heldout_lissajous_nominal_ddp.png) | ![Friction DDP: Horizontal Lissajous motion](heldout_lissajous_friction_ddp.png) | ![Joint velocity control: Horizontal Lissajous motion](heldout_lissajous_joint_velocity_control.png) | ![Slotine-Li: Horizontal Lissajous motion](heldout_lissajous_slotine_li.png) | ![DDP+GP: Horizontal Lissajous motion](heldout_lissajous_ddp_gp.png) | ![Proposed: Horizontal Lissajous motion](heldout_lissajous_proposed.png) |

## In-range payload: joint-space sinusoidal motion at 0.2 kg

These examples correspond to `Joint-space sinusoidal motion` with `payload_kg = 0.2` in
[`../payload_robustness/per_trajectory_metrics.csv`](../payload_robustness/per_trajectory_metrics.csv).

| Nominal DDP | Friction DDP | Slotine-Li | DDP+GP | Proposed |
|---|---|---|---|---|
| ![Nominal DDP: Joint-space sinusoidal motion](joint_sine_200g_nominal_ddp.png) | ![Friction DDP: Joint-space sinusoidal motion](joint_sine_200g_friction_ddp.png) | ![Slotine-Li: Joint-space sinusoidal motion](joint_sine_200g_slotine_li.png) | ![DDP+GP: Joint-space sinusoidal motion](joint_sine_200g_ddp_gp.png) | ![Proposed: Joint-space sinusoidal motion](joint_sine_200g_proposed.png) |

## Out-of-range payload: joint-space sinusoidal motion at 1.0 kg

These examples correspond to `Joint-space sinusoidal motion` with `payload_kg = 1.0` in
[`../payload_robustness/per_trajectory_metrics.csv`](../payload_robustness/per_trajectory_metrics.csv).

| Nominal DDP | Friction DDP | Slotine-Li | DDP+GP | Proposed |
|---|---|---|---|---|
| ![Nominal DDP: Joint-space sinusoidal motion](joint_sine_1kg_nominal_ddp.png) | ![Friction DDP: Joint-space sinusoidal motion](joint_sine_1kg_friction_ddp.png) | ![Slotine-Li: Joint-space sinusoidal motion](joint_sine_1kg_slotine_li.png) | ![DDP+GP: Joint-space sinusoidal motion](joint_sine_1kg_ddp_gp.png) | ![Proposed: Joint-space sinusoidal motion](joint_sine_1kg_proposed.png) |

## Out-of-range payload: circle in the XY plane at 1.0 kg

These examples correspond to `Circle in the XY plane` with `payload_kg = 1.0` in
[`../payload_robustness/per_trajectory_metrics.csv`](../payload_robustness/per_trajectory_metrics.csv).

| Nominal DDP | Friction DDP | Slotine-Li | DDP+GP | Proposed |
|---|---|---|---|---|
| ![Nominal DDP: Circle in the XY plane](circle_xy_1kg_nominal_ddp.png) | ![Friction DDP: Circle in the XY plane](circle_xy_1kg_friction_ddp.png) | ![Slotine-Li: Circle in the XY plane](circle_xy_1kg_slotine_li.png) | ![DDP+GP: Circle in the XY plane](circle_xy_1kg_ddp_gp.png) | ![Proposed: Circle in the XY plane](circle_xy_1kg_proposed.png) |

## Reproduce the added images

From the repository root:

```bash
python processing/plot_trajectory_examples.py
```

This command reads only [`adaptive_mean_curves.csv`](adaptive_mean_curves.csv)
and regenerates the 14 Slotine-Li and DDP+GP PNG files. It does not require
raw logs or modify the original controller images.
[`adaptive_curve_provenance.json`](adaptive_curve_provenance.json) records
source-export hashes, repeat counts, shared time windows, and the original
full-window EE metrics for each example. Interpolation uses a 25-ms grid
restricted to the common recorded interval and never extrapolates.

The mean-curve CSV stores measured and reference positions (`x_m`, `y_m`,
`z_m` and their `_ref_m` counterparts), `ee_position_error_mm`, and
`ee_orientation_error_deg`. The additional `ee_orientation_rx_deg`,
`ee_orientation_ry_deg`, and `ee_orientation_rz_deg` columns store mean signed
rotation-vector components; `ee_orientation_max_abs_component_deg` stores
the mean per-run maximum absolute component. All orientation values use degrees.

For maintainers who have the original processed batch exports, the optional
export step is:

```bash
python processing/export_trajectory_examples.py --source-root /path/to/processed-batches
```

The source folder contains `analysis_20261118_all_0g`,
`analysis_20261118_all_200g`, and `analysis_20261118_all_1000g`, each with
`plot_series_25ms.csv.gz` and `per_trajectory.csv`. The export step does not
read or copy raw robot logs. Repository users should use the already released
mean curves for ordinary reproduction.
