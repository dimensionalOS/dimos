from typing import Optional

from .rdata import Rdata


class Rdataset:
    ...


def from_rdata(ttl: int, *rdatas: Rdata) -> Rdataset:
    ...
