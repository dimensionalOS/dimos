# Recorders preserve observations

A live consumer may choose latest-wins delivery when stale sensor data has no value. A recording has a different contract: the recording path preserves every source observation, and the Recorder preserves every received observation, or it fails visibly. Silent coalescing cannot satisfy recording fidelity, even when bounded queues, encoder cost, database contention, or storage bandwidth make lossless operation harder.
