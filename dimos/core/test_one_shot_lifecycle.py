# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from __future__ import annotations

import threading
from unittest.mock import MagicMock

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ONE_SHOT_RPC_TIMEOUT, ModuleCoordinator
from dimos.core.module import Module, OneShotModule


class Provider(Module):
    pass


class Job(OneShotModule):
    pass


def _coordinator() -> ModuleCoordinator:
    coordinator = ModuleCoordinator.__new__(ModuleCoordinator)
    coordinator._coordinator_rpc = None
    coordinator._deployed_modules = {}
    coordinator._managers = {}
    coordinator._modules_lock = threading.RLock()
    coordinator._stop_event = threading.Event()
    coordinator._stop_lock = threading.Lock()
    coordinator._stopped = False
    return coordinator


def test_blueprint_detects_one_shot_without_instantiation() -> None:
    blueprint = autoconnect(Provider.blueprint(), Job.blueprint())
    assert blueprint.one_shot_modules == (Job,)


def test_coordinator_invokes_one_shot_exactly_once() -> None:
    coordinator = _coordinator()
    job = MagicMock()
    coordinator.get_instance = MagicMock(return_value=job)

    coordinator.run_one_shot(Job.blueprint())

    job.run_once.assert_called_once_with(rpc_timeout=ONE_SHOT_RPC_TIMEOUT)


def test_coordinator_uses_explicit_one_shot_timeout() -> None:
    coordinator = _coordinator()
    job = MagicMock()
    coordinator.get_instance = MagicMock(return_value=job)

    coordinator.run_one_shot(Job.blueprint(), rpc_timeout=42.0)

    job.run_once.assert_called_once_with(rpc_timeout=42.0)


def test_coordinator_rejects_multiple_one_shot_modules() -> None:
    coordinator = _coordinator()
    blueprint = autoconnect(Job.blueprint(), type("OtherJob", (OneShotModule,), {}).blueprint())

    with pytest.raises(ValueError, match="at most one OneShotModule"):
        coordinator.run_one_shot(blueprint)


def test_loop_unblocks_and_stop_is_idempotent() -> None:
    coordinator = _coordinator()
    thread = threading.Thread(target=coordinator.loop)
    thread.start()
    assert thread.is_alive()

    coordinator.stop()
    coordinator.stop()
    thread.join(timeout=1)

    assert not thread.is_alive()
