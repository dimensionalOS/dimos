import json
from abc import ABC, abstractmethod
from typing import Any, Callable, Generic, TypeVar

MsgT = TypeVar("MsgT")
EncodingT = TypeVar("EncodingT")


class Encoder(ABC, Generic[MsgT, EncodingT]):
    """Base class for message encoders/decoders."""

    @staticmethod
    @abstractmethod
    def encode(msg: MsgT) -> EncodingT:
        raise NotImplementedError("Subclasses must implement this method.")

    @staticmethod
    @abstractmethod
    def decode(data: EncodingT) -> MsgT:
        raise NotImplementedError("Subclasses must implement this method.")


class JSONEncoder:
    @staticmethod
    def encode(msg: MsgT) -> str:
        return json.dumps(msg).encode("utf-8")

    @staticmethod
    def decode(data: str) -> MsgT:
        return json.loads(data.decode("utf-8"))
