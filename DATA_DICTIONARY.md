# Data dictionary

## General conventions

- `method` uses the method or cumulative configuration name printed in the
  paper.
- `evaluation` numbers the three repeated evaluations used to compute the
  reported mean and sample standard deviation.
- `set` is either `Training` or `Held-out`, as in Fig. 5 and Table V.
- `trajectory` is a descriptive trajectory name. Held-out names match the five
  tasks listed in the hardware setup section of the paper.
- `RMS` denotes root mean square. Angles are in degrees unless otherwise noted.
- End-effector position errors are in millimeters.
- Torque, torque rate, and torque second derivative use Nm, Nm/s, and
  Nm/s^2, respectively.
- Hardware metric rows average repeated runs within a trajectory. Set means
  then weight trajectories equally. Summary `_std` columns are sample standard
  deviations across these trajectory means.

## Learning-curve files

| Column | Meaning |
|---|---|
| `method` | `Direct RL`, `Proposed`, or a cumulative ablation configuration. |
| `training_transitions` | Cumulative environment transitions. |
| `mean_evaluation_return` | Mean return over the evaluation trajectory set. |

## Simulation evaluation files

The controller comparison contains:

| Column | Meaning |
|---|---|
| `method` | `Nominal DDP`, `Direct RL`, or `Proposed`. |
| `evaluation` | Evaluation number, from 1 to 3. |
| `training_transitions` | Training budget used by the evaluated controller. |
| `mean_step_reward` | Mean reward per simulation step. |
| `joint_position_rms_deg` | RMS joint-position error. |
| `ee_position_rms_mm` | RMS end-effector position error. |
| `ee_orientation_rms_deg` | RMS end-effector orientation error. |

Table II EE errors use forward kinematics at matching 100-Hz timestamps.
Each evaluation averages the 21 trajectory metrics equally. Table IV retains
its logged simulator EE poses and pools time samples across trajectories for
RMS metrics. These are the processing conventions of the released data.

The component-ablation file additionally contains:

| Column | Meaning |
|---|---|
| `joint_velocity_rms_deg_s` | RMS joint-velocity error. |
| `residual_torque_rms_nm` | RMS residual-torque magnitude. |
| `residual_torque_rate_rms_nm_s` | RMS residual-torque rate. |

The cumulative names `W.`, `Rec.`, and `ID` denote reward shaping with DDP
value information, the recurrent encoder, and inverse-dynamics supervision,
respectively, as defined in Table III of the paper.

## Hardware tracking and torque file (Fig. 5)

| Column | Meaning |
|---|---|
| `set` | Training or held-out trajectory set. |
| `method` | Nominal DDP, Friction DDP, Slotine-Li, DDP+GP, Joint velocity control, or Proposed. |
| `trajectory` | Descriptive trajectory name. |
| `ee_position_mean_mm`, `ee_position_max_mm` | Mean and maximum end-effector position error. |
| `ee_orientation_mean_deg` | Mean end-effector orientation error. |
| `joint_position_rms_deg` | Joint-position RMS error averaged over joints. |
| `joint_velocity_rms_deg_s` | Joint-velocity RMS error averaged over joints. |
| `measured_torque_rms_nm` | Measured joint-torque RMS averaged over joints. |
| `measured_torque_rate_rms_nm_s` | Measured joint-torque-rate RMS averaged over joints. |
| `measured_torque_rate_p95_nm_s` | 95th percentile of absolute measured torque rate, averaged over joints. |
| `measured_torque_rate_p99_nm_s` | 99th percentile of absolute measured torque rate, averaged over joints. |
| `measured_torque_rate_peak_nm_s` | Peak absolute measured torque rate. |

Fig. 5 uses joint-position and velocity RMSE, mean EE
position and orientation error, measured-torque RMS, and measured-torque-rate
RMS. The remaining columns are retained supplementary metrics from the
original schema. EE errors are time means of Euclidean position distance and
relative-rotation angle. Measured torque rates use finite differences at the
recorded timestamps before per-joint RMS and seven-joint averaging.

As reported in the manuscript, Fig. 5 presents the comparison with no added
payload. The aggregated output is
[`figure_5_tracking_torque_summary.csv`](reproduced/tables/figure_5_tracking_torque_summary.csv).

## Hardware circular-trajectory data (Fig. 6)

The processed curves and summary metrics under
[`hardware/circular_trajectory/`](hardware/circular_trajectory/README.md)
reproduce the circular-trajectory XZ projection, enlarged three-dimensional
view, and EE position-error curve. As reported in the manuscript, this
comparison uses no added payload.

`plotted_mean_curves.csv` contains:

| Column | Meaning |
|---|---|
| `method` | Source method key; the summary file maps it to a paper-facing `label`. |
| `time_s` | Elapsed command time in seconds. |
| `n_runs` | Number of trial curves averaged. |
| `x_m`, `y_m`, `z_m` | Mean measured EE position in meters. |
| `x_ref_m`, `y_ref_m`, `z_ref_m` | Each method's mean recorded-reference EE position in meters. |
| `ee_mm` | Mean of the per-run scalar EE position errors after time interpolation, in mm. |

Each method has 398 display samples on a 25-ms grid from 0.025 to 9.95 s.
Slotine–Li and DDP+GP average three runs; the other methods use one run each.
The `ee_mm` column is not the norm of the difference between the averaged
Cartesian paths. Runtime reference timing differs between methods; the
shared reference line in the spatial panels represents the geometric path.

`selected_case_metrics.csv` contains `method`, its paper-facing `label`,
`n_runs`, full-window mean EE position error `ee_mean_mm`, mean EE orientation
error `ori_mean_deg`, and joint-position RMSE `joint_position_rmse_deg`.
These are the archived full-rate summary metrics and must not be recomputed
from the reduced display curves. The source method keys are `pure_ddp_no_rl`,
`friction`, `slotine_li`, `ddp_gp`, `velocity_control`, and `ab05_residual_rl`.

## Hardware dual-endpoint file (Table V)

| Column | Meaning |
|---|---|
| `set`, `method`, `trajectory` | Evaluation set, method, and trajectory. |
| `ee_position_mean_mm` | Mean end-effector position error. |
| `joint_position_rms_deg` | Joint-position RMS error averaged over joints. |
| `joint_velocity_rms_deg_s` | Joint-velocity RMS error averaged over joints. |
| `commanded_torque_rate_rms_nm_s` | Commanded joint-torque-rate RMS averaged over joints. |
| `measured_torque_rate_rms_nm_s` | Measured joint-torque-rate RMS averaged over joints. |
| `measured_torque_second_derivative_rms_nm_s2` | RMS second derivative of measured joint torque, averaged over joints. |

Table V compares ZOH, which holds the first filtered endpoint for each 10-ms
policy interval, with interpolation between the two filtered endpoints at
1-ms torque updates. All other controller and experimental settings are
identical. The aggregated output is
[`table_v_dual_endpoint_ablation.csv`](reproduced/tables/table_v_dual_endpoint_ablation.csv).

## Hardware payload file

| Column | Meaning |
|---|---|
| `method` | `Nominal DDP`, `Friction DDP`, `Slotine-Li`, `DDP+GP`, or `Proposed`. |
| `payload_kg` | Attached payload mass in kilograms. |
| `trajectory` | Training trajectory. |
| `ee_position_mean_mm` | Mean end-effector position error. |
| `ee_orientation_mean_deg` | Mean end-effector orientation error. |
| `joint_position_rms_deg` | Joint-position RMS error averaged over joints. |

Fig. 7 averages the same 19 training trajectories for each of the five
methods at 0, 0.2, 0.6, and 1.0 kg. For the added baselines, each zero-mass
trajectory averages three runs and each loaded trajectory uses one run.

## Hardware trajectory images

The PNG files under `hardware/trajectory_examples/` visualize matched trials
from the hardware tracking and payload files. The gallery covers spatial and
planar position tracking, an orientation-swing task, a held-out Lissajous
trajectory, and payload tests at 0.2 and 1.0 kg. Filenames identify the
paper-facing method and condition. These images are illustrative outputs from
the corresponding CSV rows and are not additional inputs to the aggregation
script. Reduced plotting summaries for Slotine–Li and DDP+GP, their field
definitions, and the generation command are documented in
[`hardware/trajectory_examples/README.md`](hardware/trajectory_examples/README.md).
The original controller images are retained from the earlier release.
