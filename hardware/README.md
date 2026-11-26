# Real-robot data

This directory contains the per-trajectory metrics used in Table V and
Figs. 4 and 6, together with the processed circular-trajectory curves for
Fig. 5. Tracking errors are computed relative to the reference trajectory.
Torque quantities distinguish the commanded joint torque from the measured
joint torque, following the notation and metric definitions in the paper.

Each metric row represents one trajectory under one method and condition.
Repeated runs are averaged first; trajectories then receive equal weight.
Tracking and measured-torque metrics are evaluated over the motion window.
Joint metrics are computed per joint and averaged over seven joints. EE
metrics use time-aligned forward kinematics. Measured torque rates use finite
differences and recorded timestamps, as in the manuscript.

The [`trajectory_examples`](trajectory_examples/) directory contains direct
reference/measurement plots for conditions included in Figs. 4 and 6.
The [`circular_trajectory`](circular_trajectory/) directory supplies the
processed curves and summary metrics for Fig. 5. As reported in the
manuscript, Figs. 4 and 5 describe comparisons with no added payload.

Table V compares holding the first filtered residual endpoint constant for
each 10-ms policy interval against interpolation between the two filtered
endpoints at each 1-ms torque update. All other controller and experimental
settings are identical. Its data are in [`dual_endpoint`](dual_endpoint/).

The [`motion_replays`](motion_replays/) directory presents selected recorded
joint motions as MuJoCo kinematic animations of all six Fig. 4 controllers:
Nominal DDP, Friction DDP, Slotine–Li, DDP+GP, Joint velocity control, and
Proposed. A three-column, two-row layout shows all methods together, with a
distinct color for each method and synchronized tracking-error curves.
These are visual reconstructions of real-robot data;
each panel uses the measured motion and recorded reference from its own trial.
The directory includes three MP4 videos, GIF previews, 18 reduced joint-trace
CSV files (one per method and trajectory), and
provenance describing the source records and rendering procedure.
Five methods use the archived plotting traces. Friction DDP traces were
extracted separately from their original ROS1 bag records and checked against
the full-rate comparison results before reduction. All 18 playback CSVs are
included, so rendering does not require the original archives or robot logs.
