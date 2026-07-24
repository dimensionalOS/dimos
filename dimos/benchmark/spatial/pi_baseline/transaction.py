# Copyright 2026 Dimensional Inc.
"""First-valid immutable answer transaction with receipt-only output."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import stat

from dimos.benchmark.spatial.models import AnswerType

from .records import Prediction
from .topology import PinnedDirectory, TopologyError


@dataclass(frozen=True)
class AnswerReceipt:
    accepted: bool
    instance_id: str
    answer_type: AnswerType


class AnswerTransaction:
    def __init__(
        self,
        instance_id: str,
        answer_type: AnswerType,
        private: PinnedDirectory | Path | None = None,
        filename: str = "prediction.provisional.v1.json",
    ) -> None:
        self.instance_id = instance_id
        self.answer_type = answer_type
        self._owned_private = False
        if isinstance(private, Path):
            private_parent = private.parent
            private_filename = private.name
            self.private = None
            self.filename = private_filename
        else:
            private_parent = None
            self.private = private
            self.filename = filename
        if self.filename in ("", ".", "..") or "/" in self.filename:
            raise TopologyError("prediction filename must be a relative child name")
        if private_parent is not None:
            self.private = PinnedDirectory.open(private_parent, create=True)
            self._owned_private = True
        self._prediction: Prediction | None = None
        if self.private is not None:
            try:
                fd = os.open(self.filename, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=self.private.fd)
            except FileNotFoundError:
                pass
            else:
                try:
                    if not stat.S_ISREG(os.fstat(fd).st_mode):
                        raise ValueError("durable prediction is not a regular file")
                    stored = Prediction.model_validate_json(_read_fd(fd))
                    if stored.instance_id != instance_id or stored.answer_type is not answer_type:
                        raise ValueError("durable prediction does not match this transaction")
                    self._prediction = stored
                finally:
                    os.close(fd)

    def submit(self, value: bool | int) -> AnswerReceipt:
        """Accept the first correctly typed answer; return a receipt only."""
        if self._prediction is not None:
            return AnswerReceipt(False, self.instance_id, self.answer_type)
        try:
            prediction = Prediction.typed(self.instance_id, self.answer_type, value)
        except ValueError:
            return AnswerReceipt(False, self.instance_id, self.answer_type)
        if self.private is not None:
            payload = prediction.model_dump_json().encode("utf-8")
            try:
                descriptor = os.open(
                    self.filename,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                    0o600,
                    dir_fd=self.private.fd,
                )
            except FileExistsError:
                descriptor = os.open(
                    self.filename, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=self.private.fd
                )
                try:
                    self._prediction = Prediction.model_validate_json(_read_fd(descriptor))
                finally:
                    os.close(descriptor)
                return AnswerReceipt(False, self.instance_id, self.answer_type)
            try:
                view = memoryview(payload)
                while view:
                    view = view[os.write(descriptor, view) :]
                os.fsync(descriptor)
                os.fsync(self.private.fd)
            except BaseException:
                try:
                    os.unlink(self.filename, dir_fd=self.private.fd)
                except FileNotFoundError:
                    pass
                raise
            finally:
                os.close(descriptor)
        self._prediction = prediction
        return AnswerReceipt(True, self.instance_id, self.answer_type)

    @property
    def prediction(self) -> Prediction | None:
        return self._prediction

    @property
    def provisional_filename(self) -> str:
        """Return the private, pre-commit answer filename."""
        return self.filename

    def close(self) -> None:
        if self._owned_private:
            assert self.private is not None
            self.private.close()
            self._owned_private = False

    def __enter__(self) -> AnswerTransaction:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        if getattr(self, "_owned_private", False):
            self.close()


def _read_fd(fd: int) -> bytes:
    chunks: list[bytes] = []
    while True:
        chunk = os.read(fd, 64 * 1024)
        if not chunk:
            return b"".join(chunks)
        chunks.append(chunk)
