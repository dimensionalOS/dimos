FIT  g1_groot_characterization_2026-08-27.db  preset stock  schedule seed 0

PINNED (not searched — provenance, not optimism)
  damping                       0.00100   unresolved without a hanging recording; the fork agrees it is small
  foot_friction                 1.00000   a floor property; the recording's floor is undeclared
  foot_friction_torsional       0.00500   the patch formula is meaningless for a four-sphere foot
  foot_solimp_dmin              0.90000   contact shape; not resolved by walking, stock stands
  foot_solimp_width             0.00100   contact shape; not resolved by walking, stock stands
  leg_mass_scale                1.00000   declared: CAD masses from Unitree's URDF
  trunk_com_x                   0.00000   declared: not weighed
  trunk_inertia_scale           1.00000   declared: not weighed
  trunk_mass_scale              1.00000   declared: not weighed (model mass 35.112 kg)

SEARCHED  within the knob's declared range, in its own metric
  armature                   [0.002, 0.05] log   stock 0.01; fork 0.0138; Go2 spread up to 0.049
  frictionloss               [0.05, 8] log   stock 0.1; fork 3.25 (32x stock, outside the Go2 0.1-2.0); sealed high-ratio planetary drives make several N.m plausible
  actuator_tau               [0.0005, 0.02] log   the Go2 range; nothing G1-specific measured yet
  foot_solref_time           [0.002, 0.03] log   stock 0.02; below ~0.01 is stiff for the 5 ms step
  foot_solref_damp           [0.5, 2] lin   stock 1.0; the Go2 range

POINT AND SPREAD  median and 10th-90th of 14 pooled near-optimal trials from 12 studies — never the best draw
  knob                            point         p10 .. p90         of range
  armature                      0.01756     0.00630 .. 0.02879        47.2%
  frictionloss                  6.58212     5.02487 .. 7.74321         8.5%
  actuator_tau                  0.00236     0.00182 .. 0.00539        29.5%
  foot_solref_time              0.00673     0.00396 .. 0.01209        41.2%
  foot_solref_damp              1.24696     1.06607 .. 1.58699        34.7%

STUDIES  harvest = trials within 1 paired SE of the study's best; per-seed bests are DIAGNOSTIC — the shipped point is the median
  seed trials       best  vs base harvest       tol
     0     30    0.67523   -17.2%       2   0.00733
     1     30    0.66737   -18.2%       1   0.00832
     2     30    0.66741   -18.2%       1   0.00690
     3     30    0.67734   -16.9%       1   0.00718
     4     30    0.66024   -19.0%       1   0.00862
     5     30    0.67449   -17.3%       2   0.00593
     6     30    0.67389   -17.4%       1   0.00647
     7     30    0.67950   -16.7%       1   0.00820
     8     30    0.67312   -17.5%       1   0.00623
     9     30    0.67145   -17.7%       1   0.00508
    10     30    0.66546   -18.4%       1   0.00631
    11     30    0.69577   -14.7%       1   0.00359

  seed sensitivity of the best draw (the reason the median ships):
    armature                    0.00618 .. 0.04435    (7.2x across seeds)
    frictionloss                3.25291 .. 7.85805    (2.4x across seeds)
    actuator_tau                0.00065 .. 0.00635    (9.7x across seeds)
    foot_solref_time            0.00272 .. 0.01312    (4.8x across seeds)
    foot_solref_damp            1.02627 .. 1.69103    (1.6x across seeds)

STOPPING  leave-one-study-out spread drift (below tolerance twice in a row stops)
  k=3: 0.980  k=4: 0.657  k=5: 0.587  k=6: 0.623  k=7: 0.368  k=8: 0.348  k=9: 0.244  k=10: 0.252  k=11: 0.268  k=12: 0.201
  HIT THE STUDY CAP without stabilising: the region is wider than this data can pin. That is the result — ship the spread, not a point.

SCORE  weighted (channel, regime) residuals / baseline scales
  baseline 0.81555 -> point 0.67097   -17.7%

CHANNELS  every residual the data can answer, scored or not — the
zero-weight rows are the misspecification map, not omissions
  channel/regime        weight   baseline   -> value unit     /scale  share  scored?
  accel/floor             0.00     1.5505     1.4929 m/s^2      0.69      —  SHOWN, not scored
  dq/floor                0.30     0.3912     0.2884 rad/s      0.53  31.4%  yes
  gyro/floor              0.00     0.3520     0.3069 rad/s      0.71      —  SHOWN, not scored
  joint/floor             0.30     0.0436     0.0362 rad        0.74  44.1%  yes
  pos/floor               0.00     0.0618     0.0550 m          0.55      —  SHOWN, not scored
  rot/floor               0.00     0.1098     0.0931 rad        0.57      —  SHOWN, not scored
  tau/floor               0.15     3.3737     3.2720 N*m        0.83  24.4%  yes
