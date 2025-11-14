from typing import List, Optional, Union

from .name import Name
from .rrset import RRset


class Message:
    id: int
    flags: int
    answer: List[RRset]
    question: List[RRset]

    def __init__(self, id: Optional[int] = None) -> None:
        ...

    def to_wire(self) -> bytes:
        ...

    ...


class QueryMessage(Message):
    ...


def from_wire(wire: bytes) -> Message:
    ...


def make_query(qname: Union[Name, str], rdtype: Union[int, str]) -> QueryMessage:
    ...
