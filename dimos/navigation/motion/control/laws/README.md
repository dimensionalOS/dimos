# laws

One module per law; a track names one ([`../tracks.py`](../tracks.py)).

`seed` is the permanent baseline — every track's A/B is against it and every
search seeds from it, so it does not absorb research results. A research
generation lands by replacing its own track's module and flipping the one line
in `TRACKS` that names it.

Each `make_rust` law is a port of its python twin, not a variant:
`test_rust_parity.py` holds every pair to 1e-9 per twist component.
