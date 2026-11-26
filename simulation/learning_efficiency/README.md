# Learning efficiency (Fig. 1)

`learning_curves.csv` contains the mean evaluation return versus cumulative
environment transitions for Direct RL and Proposed. Direct RL was continued to
100 million transitions, whereas Proposed was trained for 10 million
transitions, as described in the paper. The curves retain their respective
training objectives, which penalize different torque signals. Their return
magnitudes are therefore not a common tracking-accuracy metric. All recorded
evaluations within each reported training horizon are included without
smoothing. The Direct RL series follows
the continuation lineage used to reach the reported budget; overlapping log
entries at a resumed boundary appear only once. The final continuation resumes
at its logged 80-million-transition budget from the preceding checkpoint, so
the CSV contains no records between 79.2 and 80.1 million transitions and no
values are interpolated across that interval.

The Fig. 1 display follows the manuscript: its logarithmic horizontal axis
spans 0.55–110 million transitions, and the Proposed curve is shown from
the common starting point of 0.6 million transitions. Direct RL is plotted
in separate continuation segments; after 10 million transitions,
approximately one point per million is shown,
with segment endpoints retained. The CSV still includes every recorded
evaluation, and the plot does not connect the 79.2–80.1 million gap.
Shading shows the gap over the shared training interval and the difference
between the two peak returns. The percentage annotations use the recorded
values at 10 million transitions and the full-record peaks, respectively.
