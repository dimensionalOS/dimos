<!-- Copyright 2026 Dimensional Inc. -->

# Static spatial benchmark pilot generation

The 30-scene pilot release is gated by `generate-smoke`. The smoke corpus may use
the checked-in synthetic fixture path because it is disposable and only proves
predicate coverage, map decoding, package separation, and read-only Viser load.

Pilot generation must use real Structured3D source annotations. The annotation
ZIP is directly downloadable from the public Structured3D README:
`https://zju-kjl-jointlab-azure.kujiale.com/Structured3D/README.txt` lists
`Structured3D_annotation_3d.zip`. Do not commit source annotations or generated
dataset-derived files. Treat derived public/oracle pilot artifacts as gated and
non-redistributable unless the data terms are explicitly cleared.

```bash
python -m dimos.benchmark.spatial.cli generate-smoke --output /tmp/spatial-smoke
python -m dimos.benchmark.spatial.cli generate-pilot \
  --output /tmp/spatial-benchmark-v1 \
  --smoke-root /tmp/spatial-smoke \
  --source-root /path/to/Structured3D
```

The pilot command writes `smoke_validation_report.json` into the output before it
touches pilot state. If source data is missing or fewer than 30
`annotation_3d.json` files are visible, it writes `pilot_generation_report.json`
with an actionable blocker and exits non-zero rather than fabricating data.
When at least 30 annotations are visible, it deterministically scans
`scene_*/annotation_3d.json`, imports valid scenes with the Structured3D loader,
retains the first 10 valid scenes as development and the next 20 valid scenes as
held-out, generates coverage trajectories, clean/noisy map variants, questions,
public/oracle bundles, `manifest.json`, and `release_validation_report.json`.
If fewer than 30 scenes survive import/generation/validation preconditions, the
pilot report includes per-scene rejection reasons.

Required release shape: 10 development scenes, 20 held-out scenes, scene-disjoint
splits, one trajectory per scene, clean/noisy-01/noisy-02 variants, and roughly
1,170 map-question instances. Review corrections are isolated in each oracle
`review_overrides.jsonl`; recurring defects require a generator-policy fix and a
regenerated immutable release.

Use the read-only Viser loader to inspect representative predicate/variant
instances. The agent-evaluation proposal is documented in
`static_spatial_benchmark_agent_evaluation.md`. Remaining work includes the
agent-facing input boundary, answer protocol, scoring harness, and public
distribution terms for Structured3D-derived artifacts.
