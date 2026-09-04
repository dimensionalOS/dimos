# Vendored ABC inference code

`abc_minimal/dit.py`, `preprocess.py`, and `fast_inference.py` come from
[`amazon-far/abc`](https://github.com/amazon-far/abc) at revision
`6bc6586721cf0c409ccee80f675a28de9b9b2f5e`. `config.py` retains only the two
configuration dataclasses used for inference, and `fast_inference.py` omits the
RTC helper. The upstream project is licensed under Apache-2.0, the same license
as this repository.

Training, simulation, dataset conversion, visualization, and RTC code are not
vendored.
