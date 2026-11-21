"""Killable CPU actor subprocess for the native Quest task only.

No SDK, bus or control state enters this process. One observation/result in
flight; model warm-up completes before task construction. On deadline failure
it is quarantined until stack restart (no automatic new worker while held).
"""

from __future__ import annotations

import multiprocessing
from multiprocessing.connection import Connection
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from dimos.control.tasks.policy_execution import OnnxActor


class BoundedNativeActor(OnnxActor):
    def __init__(self, model: Path, *, timeout_s: float = 0.020) -> None:
        self.timeout_s = timeout_s
        self.failed = False
        parent, child = multiprocessing.Pipe()
        self._pipe = parent
        self._process = subprocess.Popen(
            [sys.executable, "-m", __name__, str(child.fileno()), str(model)],
            pass_fds=(child.fileno(),),
            stdin=subprocess.DEVNULL,
        )
        child.close()
        started = time.perf_counter()
        try:
            if not parent.poll(30):
                raise TimeoutError("native actor startup/warm-up deadline")
            ready = parent.recv()
            if ready.get("ready") is not True:
                raise RuntimeError(f"native actor failed: {ready}")
            self._providers = ready["providers"]
            self.warmup_s = ready["warmup_s"]
            self.startup_s = time.perf_counter() - started
        except BaseException:
            self.close()
            raise

    @property
    def providers(self) -> list[str]:
        return list(self._providers)

    def run(self, observation: np.ndarray) -> np.ndarray:
        if self.failed or self._process.poll() is not None:
            raise RuntimeError("native actor quarantined; restart stack while released")
        started = time.perf_counter()
        try:
            self._pipe.send(np.ascontiguousarray(observation, dtype=np.float32))
            remaining = self.timeout_s - (time.perf_counter() - started)
            if remaining <= 0 or not self._pipe.poll(remaining):
                raise TimeoutError("native actor submission-to-result deadline")
            result = self._pipe.recv()
            if time.perf_counter() - started > self.timeout_s:
                raise TimeoutError("native actor late result")
            if isinstance(result, dict):
                raise RuntimeError(result)
            return np.asarray(result, dtype=np.float32)
        except BaseException:
            self.failed = True
            # Killing/reaping is deferred to stop: no join on the control tick.
            if self._process.poll() is None:
                self._process.kill()
            raise

    def close(self) -> None:
        self.failed = True
        if self._process.poll() is None:
            self._process.kill()
        self._process.wait(timeout=2)
        self._pipe.close()


def _serve(fd: int, model: str) -> None:
    pipe = Connection(fd)
    try:
        actor = OnnxActor(
            model, providers=["CPUExecutionProvider"], input_name="obs", output_name="action"
        )
        start = time.perf_counter()
        for _ in range(3):
            actor.run(np.zeros((1, 339), dtype=np.float32))
        pipe.send(
            {"ready": True, "providers": actor.providers, "warmup_s": time.perf_counter() - start}
        )
        while True:
            obs = pipe.recv()
            pipe.send(actor.run(obs))
    except EOFError:
        pass
    except Exception as error:
        pipe.send({"error": repr(error)})
    finally:
        pipe.close()


if __name__ == "__main__":
    _serve(int(sys.argv[1]), sys.argv[2])
