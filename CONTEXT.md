# DimOS

DimOS composes typed data streams into robot systems. This glossary names the observations and guarantees involved in recording those streams.

## Recording

**Source observation**:
An observation whose publisher call completed successfully.
_Avoid_: Frame, sent message

**Received observation**:
A source observation emitted by the Recorder's input transport before Recorder scheduling or storage.
_Avoid_: Recorded observation

**Persisted observation**:
An observation committed to the recording and readable through its configured codec.
_Avoid_: Received observation, saved frame

**Recording fidelity**:
Exact correspondence between source observations and persisted observations, including membership, order, timestamps, and codec-defined payload content.
_Avoid_: Frame rate, throughput

**Recorder fidelity**:
Exact correspondence between received observations and persisted observations. Recording fidelity also includes the transport path before the Recorder.
_Avoid_: Recording fidelity

**Shared loss window**:
A source-time interval in which every recorded data stream is missing observations.
_Avoid_: Freeze, lag spike

**Tail loss**:
Observations lost between the start of graceful shutdown and the recording's final commit.
_Avoid_: Shared loss window
