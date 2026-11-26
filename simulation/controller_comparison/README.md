# Simulation controller comparison (Table II)

`evaluation_metrics.csv` contains one row for each of the three evaluations of
Nominal DDP, Direct RL, and Proposed. The columns are the four metrics reported
in Table II. Direct RL is evaluated after its extended training budget, while
Proposed uses the training budget specified for residual policies.
Learned controllers use the checkpoint with the highest stored mean evaluation
return within the corresponding budget.

EE position and orientation errors use forward kinematics of measured and
reference joint positions at matching 100-Hz timestamps. These two columns
replace the earlier cached-pose metrics and match the revised Table II.
The stored rewards and joint-position errors are unchanged. Each evaluation
weights the 21 trajectories equally; evaluations 1–3 correspond to seeds
42–44.

`processing/reproduce_results.py` computes the mean and sample standard
deviation over the three evaluations.
