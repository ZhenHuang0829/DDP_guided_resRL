# Hardware tracking and torque comparison (Fig. 5)

`per_trajectory_metrics.csv` contains the training and held-out results for
Nominal DDP, Friction DDP, Slotine–Li, DDP+GP, Joint velocity control, and
Proposed. Fig. 5 reports six metrics: joint-position and velocity
RMSE, mean EE position and orientation error, and measured-torque and
torque-rate RMS. Additional columns retained from the earlier release are
not used in the current figure. The aggregated output is
[`figure_5_tracking_torque_summary.csv`](../../reproduced/tables/figure_5_tracking_torque_summary.csv).

Each method has 19 training and five held-out trajectory rows. For the added
baselines, each row averages three runs, except DDP+GP on the held-out joint
chirp, which averages six. Set means weight trajectories equally, so the
extra chirp repeats do not increase its weight. The exported standard
deviations describe variation across trajectory means, not across all runs.

As reported in the manuscript, the comparison uses no added payload.
