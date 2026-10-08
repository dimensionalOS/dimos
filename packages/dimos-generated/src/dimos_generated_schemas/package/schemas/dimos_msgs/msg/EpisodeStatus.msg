# EpisodeStatus internal model from robot-learning PR4343, ef5e5f2710c482fb1d43d43ec50fd73b0fbe1dde.
# Seconds; application validation requires a finite value.
float64 ts
# Application values: idle, recording. Required by the source model.
string state
# Source Python ints use signed 64-bit wire storage; negative values are retained.
int64 episodes_saved
int64 episodes_discarded
# Application values: start, save, discard, init.
string last_event "init"
# Nullable string: [] means None; [""] preserves an explicitly empty label.
string[<=1] task_label
