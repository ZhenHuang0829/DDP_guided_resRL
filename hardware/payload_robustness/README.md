# Robustness to unmodeled payloads (Fig. 7)

`per_trajectory_metrics.csv` contains the three tracking metrics plotted in
Fig. 7 for Nominal DDP, Friction DDP, Slotine–Li, DDP+GP, and Proposed at attached payload masses
of 0, 0.2, 0.6, and 1.0 kg. Each method-payload combination uses the same
training trajectory set.

Each row is a trajectory mean. For the added baselines, the zero-mass rows
average three runs and each loaded row represents one run. The plotted
means give equal weight to all 19 trajectories. Held-out trajectories are not
included in this payload comparison.

The training payload-mass range is 0–0.8 kg. The 0.2- and 0.6-kg conditions
lie within that range, whereas 1.0 kg is outside it. The shaded region in
Fig. 7 denotes masses above 0.8 kg. The nominal model
assumes no external payload in every condition, so payload mass, center of
mass, and inertia are unmodeled during control.
