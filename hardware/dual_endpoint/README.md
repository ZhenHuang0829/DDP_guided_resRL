# ZOH versus interpolated residuals on the real robot (Table V)

`per_trajectory_metrics.csv` compares the ZOH residual with Proposed on the
training and held-out sets. The six metric columns correspond directly to
Table V. The aggregated output is
[`table_v_dual_endpoint_ablation.csv`](../../reproduced/tables/table_v_dual_endpoint_ablation.csv).

During each 10-ms policy interval, the ZOH controller holds the first filtered
residual torque endpoint constant. Proposed interpolates between the two
filtered endpoints at each 1-ms torque update. All other controller and
experimental settings are identical, as described in the manuscript.
