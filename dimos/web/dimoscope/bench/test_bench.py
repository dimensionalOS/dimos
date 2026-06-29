"""CI-checkable transport benchmark: start a Bun↔LCM gateway + bench publisher,
run the headless bench, and assert latency / throughput / on-demand are sane.

    pytest dimos/web/dimoscope/bench/test_bench.py -s
"""

from __future__ import annotations

import json
import os
import pathlib
import shutil
import signal
import subprocess
import time

import pytest

HERE = pathlib.Path(__file__).resolve().parent.parent  # dimoscope/
REPO = HERE.parents[2]  # dimos repo root
PY = str(REPO / ".venv/bin/python")
PORT = "8092"  # dedicated port so a dev gateway on :8090 doesn't clash


@pytest.mark.skipif(shutil.which("bun") is None, reason="bun not installed")
def test_bench_lcm() -> None:
    procs: list[subprocess.Popen] = []
    try:
        procs.append(
            subprocess.Popen(
                ["bun", "run", "servers/gateway.ts"],
                cwd=HERE,
                env={**os.environ, "GATEWAY_PORT": PORT, "DIMOS_LCM_PORT": "7667"},
            )
        )
        procs.append(
            subprocess.Popen(
                [PY, "bench/bench_publisher.py"],
                cwd=HERE,
                env={**os.environ, "DIMOS_TRANSPORT": "lcm", "BENCH_HZ": "100"},
            )
        )
        time.sleep(3)
        subprocess.run(
            ["bun", "run", "bench/bench.ts"],
            cwd=HERE,
            env={
                **os.environ,
                "GATEWAY_URL": f"ws://localhost:{PORT}",
                "BENCH_DUR_MS": "3000",
                "BENCH_LABEL": "Bun↔LCM gateway (pytest)",
            },
            check=True,
            timeout=60,
        )
        res = json.loads((HERE / "bench" / "results.json").read_text())
        thr = next(r for r in res["rows"] if r["topics"] == 4)
        assert thr["msgs"] > 100, f"no throughput: {thr}"
        assert thr["latP50"] < 50, f"latency too high: {thr['latP50']}ms"
        assert res["ondemandSaving"] > 40, f"on-demand saving low: {res['ondemandSaving']}%"
    finally:
        for p in procs:
            p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
