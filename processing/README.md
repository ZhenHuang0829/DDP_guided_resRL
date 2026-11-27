# Data-processing code

This directory contains only the aggregation, plotting, and validation code
needed for the released data.

- `reproduce_results.py` regenerates Tables II, IV, and V and Figs. 2, 3, 5,
  6, and 7. It also calls the circular-trajectory plotter.
- `plot_circular_trajectory.py` reproduces Fig. 6 from the released processed
  circular-trajectory curves and summary metrics.
- `plot_trajectory_examples.py` regenerates the added Slotine–Li and DDP+GP
  gallery images from reduced plotting summaries.
- `validate_release.py` checks record counts, public method names, CSV schemas,
  portability, and numerical agreement with the manuscript tables.
- `render_real_robot_replays.py` renders three MuJoCo animations comparing
  all six Fig. 5 controllers in a three-column, two-row layout. It reads the
  18 released real-robot joint-trace CSVs, retaining each selected trial's
  measured motion and recorded reference. Method colors match across robot
  views and synchronized tracking-error curves.
  This optional renderer uses `requirements_replay.txt`, FFmpeg, and a Panda
  model. Ordinary rendering needs no original archive or robot logs; see the
  [replay instructions](../hardware/motion_replays/README.md).

The optional replay-data import combines the original five-method plotting
archive (`--source`, with `--source-summary`) and the separately extracted
Friction DDP traces (`--extra-source`, with `--source-evidence`). The
`--extra-source` option can be repeated for multiple supplementary exports.
These maintainer inputs are not needed to render the 18 released CSVs.

Run the analysis and validation scripts from the `github_data` directory:

```bash
python processing/reproduce_results.py
python processing/plot_trajectory_examples.py
python processing/validate_release.py
```

The required packages are listed in `../requirements.txt`.

The hardware tracking summary for Fig. 5 is
[`figure_5_tracking_torque_summary.csv`](../reproduced/tables/figure_5_tracking_torque_summary.csv).
The ZOH/interpolation comparison for Table V is
[`table_v_dual_endpoint_ablation.csv`](../reproduced/tables/table_v_dual_endpoint_ablation.csv).
