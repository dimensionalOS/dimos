# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copyright 2026 Dimensional Inc.
from datetime import datetime, timezone

from dimos.benchmark.spatial.pi_baseline.gate import (
    AUDIT_LIMITATION,
    ArtifactReference,
    HumanReleaseRecord,
    InfrastructureCheck,
    ReleaseBlockedError,
    SmokeRunEvidence,
    evaluate_release,
    require_release,
)

HASH = "a" * 64


def _ref(name: str) -> ArtifactReference:
    return ArtifactReference(path=name, sha256=HASH)


def _record(**changes: object) -> HumanReleaseRecord:
    checks = tuple(
        InfrastructureCheck(name=name, status="passed")
        for name in (
            "oauth-model-resolution",
            "image-digest",
            "rootless-isolation",
            "read-only-input",
            "oracle-absence",
            "tool-digest",
            "network-availability",
            "audit-collection",
            "resource-limits",
            "submission-immutability",
            "scorer-isolation",
            "export-integrity",
            "destruction",
        )
    )
    runs = tuple(
        SmokeRunEvidence(
            run_id=mode,
            mode=mode,
            case_sha256=HASH,
            manifest_sha256=HASH,
            review_bundle=_ref(f"{mode}.bundle"),
            private_score=_ref(f"{mode}.score"),
            transcript=_ref(f"{mode}.transcript"),
            tool_trace=_ref(f"{mode}.tools"),
            audit=_ref(f"{mode}.audit"),
        )
        for mode in ("visualization-forbidden", "visualization-encouraged")
    )
    values: dict[str, object] = dict(
        release_id="release-1",
        infrastructure=checks,
        smoke_runs=runs,
        required_review_artifact=_ref("review.json"),
        reviewer="human",
        reviewed_at=datetime.now(timezone.utc),
        decision="approved",
        audit_limitation=AUDIT_LIMITATION,
    )
    values.update(changes)
    return HumanReleaseRecord(**values)


def test_approved_record_is_releasable() -> None:
    record = _record()
    assert evaluate_release(record).permitted
    require_release(record)


def test_every_material_block_state() -> None:
    cases = (
        ({"infrastructure": ()}, "missing infrastructure check"),
        (
            {"infrastructure": (InfrastructureCheck(name="image-digest", status="failed"),)},
            "failed infrastructure check",
        ),
        ({"smoke_runs": ()}, "missing smoke mode"),
        ({"required_review_artifact": None}, "missing required review artifact"),
        ({"decision": None}, "no explicit human approval"),
        ({"reviewer": None}, "missing reviewer"),
        ({"reviewed_at": None}, "missing review timestamp"),
        ({"audit_limitation": "clean"}, "missing honest network-audit limitation"),
    )
    for changes, blocker in cases:
        result = evaluate_release(_record(**changes))
        assert not result.permitted
        assert any(blocker in item for item in result.blockers)


def test_missing_run_artifacts_and_nonidentical_case_block() -> None:
    record = _record(
        smoke_runs=(
            SmokeRunEvidence(
                run_id="one", mode="visualization-forbidden", case_sha256=HASH, manifest_sha256=HASH
            ),
            SmokeRunEvidence(
                run_id="two",
                mode="visualization-encouraged",
                case_sha256="b" * 64,
                manifest_sha256=HASH,
            ),
        )
    )
    result = evaluate_release(record)
    assert not result.permitted
    assert any("missing private score reference" in item for item in result.blockers)
    assert any("different cases" in item for item in result.blockers)


def test_require_release_raises_for_blocked_record() -> None:
    try:
        require_release(_record(decision="rejected"))
    except ReleaseBlockedError as error:
        assert "no explicit human approval" in str(error)
    else:
        raise AssertionError("blocked release was accepted")
