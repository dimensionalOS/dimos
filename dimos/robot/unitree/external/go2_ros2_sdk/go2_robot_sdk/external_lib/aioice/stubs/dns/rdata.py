from typing import Any


class Rdata:
    rdclass: int
    rdtype: int
    ...

    def to_generic(self) -> GenericRdata:
        ...


class GenericRdata(Rdata):
    data: bytes
    ...

    def __init__(self, rdclass: int, rdtype: int, data: bytes) -> None:
        ...
