# Staged component ablation (Fig. 3 and Table IV)

The labels `MLP baseline`, `MLP + W.`, `Rec. + W.`, `Rec. + W. + ID`, and
`Proposed` follow Table III of the paper.

- `learning_curves.csv` contains the training-time evaluation return used in
  Fig. 3, including every recorded evaluation point.
- `evaluation_metrics.csv` contains three post-training evaluations per
  configuration for Table IV. Each fixed checkpoint is determined by the same
  highest-stored-return rule within the common training budget.

Fig. 3 presents training-time returns, whereas Table IV reports post-training
trajectory metrics. The quantities and evaluation procedures differ and
should therefore be interpreted within the corresponding analysis. All three
post-training evaluations are included. The five configurations follow the
successive additions listed in Table III.

For data processing, the Proposed row in Table IV uses a separate
training run from Table II and Fig. 3. Table IV pools time samples across
trajectories for RMS metrics and uses the logged simulator EE poses; these
conventions differ from the time-aligned, trajectory-weighted Table II.
