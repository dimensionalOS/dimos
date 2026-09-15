# ABC inference source

Snapshot from https://github.com/amazon-far/abc/tree/6bc6586721cf0c409ccee80f675a28de9b9b2f5e.
Included: config, model/checkpoint loader, preprocessing, CUDA graph helpers, and licenses.
Training and simulation entrypoints are excluded. The package uses implicit namespace
discovery, and CUDA warmup results are evaluated without unused assignments to `_`.
These repository-style adaptations leave the numerical operations unchanged.
Behavioral changes belong in dimos_abc/backend.py.
