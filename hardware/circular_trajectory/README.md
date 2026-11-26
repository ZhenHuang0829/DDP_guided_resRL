# Circular trajectory: Figure 5

These files reproduce the manuscript's circular-trajectory comparison with no added payload, using recorded real-robot measurements. The six controllers and all available repetitions are retained: three runs each for Slotine–Li and DDP+GP, and one run each for Nominal DDP, Friction DDP, Joint velocity control, and Proposed.

The original selection rule chose the trajectory with the largest relative reduction in mean end-effector (EE) position error against the best baseline among the 19 archived training trajectories. This selected example is `panda_ddp_circle_plan`. The full-rate mean EE errors are 0.3106270589218955mm for Proposed and 0.5844279838422304mm for DDP+GP, a 46.84938649245943% reduction; the manuscript reports 0.311mm, 0.584mm, and 46.8%.

`plotted_mean_curves.csv` contains 398 samples per method (2,388 rows) on the original common 25ms grid from 0.025s to 9.95s. The source processing interpolated each run onto that grid and then averaged across all available repeats. Measured positions, reference positions, and scalar EE errors were averaged separately. In particular, `ee_mm` is the mean of the per-run scalar errors, not the distance between averaged measured and reference positions. These published curves are copied without resampling, smoothing, phase alignment, or error magnification.

Each method retains its own recorded reference in `x_ref_m`, `y_ref_m`, and `z_ref_m`; historical command timing differs between runs. The spatial panels use the Proposed reference to represent the shared path geometry, whose audited maximum separation across methods is below 0.031mm. The error panel retains each run's own reference. The 10mm detail cube is near the reference at 5s, with limits x=[242,252]mm, y=[−5,5]mm, and z=[405,415]mm.

`selected_case_metrics.csv` contains the existing full-rate per-trajectory statistics. They are distinct from averages computed over the plotted 25ms samples and common display window. The `n_runs` column records the number of included repetitions. Position columns ending in `_m` are in meters; `time_s` is in seconds; `ee_mm` and `ee_mean_mm` are in millimeters; `ori_mean_deg` and `joint_position_rmse_deg` are in degrees. Method keys follow the audited figure inputs, including `friction` for Friction DDP.

From the repository root, run:

```bash
python processing/plot_circular_trajectory.py
```

The script requires only NumPy, pandas, and Matplotlib and writes `reproduced/figures/figure_5_circular_trajectory.pdf` and `.png`. It also provides `reproduce_figure_5(figure_dir=None)` for the main reproduction script and accepts `--figure-dir` on the command line. It uses a local Agg canvas without changing the importing program's backend, and preserves the manuscript's panel proportions, axis limits, colors, line styles, legend, and annotations. Times New Roman is used when installed, with STIXGeneral as the fallback; no TeX installation is required.

`provenance.json` records byte-identical source CSV hashes, the original selection and aggregation rules, all included runs and plotting windows, reference checks, and source-renderer hashes. Historical source basenames identify audited inputs, not additional downloadable files. Reproduction does not read the historical logs or manuscript directory.
