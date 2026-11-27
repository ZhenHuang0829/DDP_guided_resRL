# Simulation data

This directory contains the data used for Figs. 2 and 3 and Tables II and IV.
The controller comparison and component ablation each contain the three
evaluations summarized by the mean and sample standard deviation in the paper.
All names for methods and cumulative component configurations follow the
manuscript.

The training curves contain all recorded evaluation points. Each learned
configuration is evaluated after applying the same checkpoint rule: the
highest stored mean evaluation return within its prescribed training budget.
The repeated evaluations characterize each fixed checkpoint rather than
independent training runs.
