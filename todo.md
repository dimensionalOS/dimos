# Triage handover — chore/dissolve-memory-dir

Review verdict: Approve (0 blockers, 0 should-fix, 1 trivial nit). Triaged → fix the nit.

## Must fix / real bugs
(none)

## Fix (agreed)
- [x] 🟡 Tidy vestigial `temp_dir` scaffolding in `dimos/utils/timeseries/test_base.py` — removed the `temp_dir` fixture + `tempfile` import, factory now `lambda: …` no-arg, dropped the param from all 25 test signatures and `store_factory()` calls. 25 pass, gate green.
  After trimming to InMemory-only, `temp_dir` is unused. Drop the `temp_dir`
  fixture, change the factory to take no arg (`lambda: make_in_memory_store()`),
  and remove the `temp_dir` param from every parametrized test method signature
  (and the `store_factory(temp_dir)` → `store_factory()` calls). Remove the now-unused
  `tempfile` import. Keep the parametrize over `testdata` (still 1 case).

## Decisions needed
(none)

## Skip
(none)
