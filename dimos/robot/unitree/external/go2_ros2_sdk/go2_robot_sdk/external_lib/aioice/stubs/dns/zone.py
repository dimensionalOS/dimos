from typing import Union

from .name import Name
from .rdataclass import IN
from .rdataset import Rdataset
from .rrset import RRset


class Zone:
    def __init__(
        self, origin: str, rdclass: int = IN, relativize: bool = False
    ) -> None:
        ...

    def find_rrset(self, name: Union[Name, str], rdtype: int) -> RRset:
        ...

    def replace_rdataset(self, name: Union[Name, str], replacement: Rdataset) -> None:
        ...

    ...
