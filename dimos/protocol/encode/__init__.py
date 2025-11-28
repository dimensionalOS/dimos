from abc import ABC, abstractmethod
import json
from typing import Generic, Protocol, TypeVar

MsgT = TypeVar("MsgT")
EncodingT = TypeVar("EncodingT")


class LCMMessage(Protocol):
    """Protocol for LCM message types that have encode/decode methods."""

    def encode(self) -> bytes:
        """Encode the message to bytes."""
        ...

    @staticmethod
    def decode(data: bytes) -> "LCMMessage":
        """Decode bytes to a message instance."""
        ...


# TypeVar for LCM message types
LCMMsgT = TypeVar("LCMMsgT", bound=LCMMessage)


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


class JSON(Encoder[MsgT, bytes]):
    @staticmethod
    def encode(msg: MsgT) -> bytes:
        return json.dumps(msg).encode("utf-8")

    @staticmethod
    def decode(data: bytes) -> MsgT:
        return json.loads(data.decode("utf-8"))  # type: ignore[no-any-return]


class LCM(Encoder[LCMMsgT, bytes]):
    """Encoder for LCM message types."""

    @staticmethod
    def encode(msg: LCMMsgT) -> bytes:
        return msg.encode()

    @staticmethod
    def decode(data: bytes) -> LCMMsgT:
        # Note: This is a generic implementation. In practice, you would need
        # to pass the specific message type to decode with. This method would
        # typically be overridden in subclasses for specific message types.
        raise NotImplementedError(
            "LCM.decode requires a specific message type. Use LCMTypedEncoder[MessageType] instead."
        )


class LCMTypedEncoder(LCM, Generic[LCMMsgT]):  # type: ignore[type-arg]
    """Typed LCM encoder for specific message types."""

    def __init__(self, message_type: type[LCMMsgT]) -> None:
        self.message_type = message_type

    @staticmethod
    def decode(data: bytes) -> LCMMsgT:
        # This is a generic implementation and should be overridden in specific instances
        raise NotImplementedError(
            "LCMTypedEncoder.decode must be overridden with a specific message type"
        )


def create_lcm_typed_encoder(message_type: type[LCMMsgT]) -> type[LCMTypedEncoder[LCMMsgT]]:
    """Factory function to create a typed LCM encoder for a specific message type."""

    class SpecificLCMEncoder(LCMTypedEncoder):  # type: ignore[type-arg]
        @staticmethod
        def decode(data: bytes) -> LCMMsgT:
            return message_type.decode(data)  # type: ignore[return-value]

    return SpecificLCMEncoder
