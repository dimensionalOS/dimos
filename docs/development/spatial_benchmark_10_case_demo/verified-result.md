# Verified 10-case result

The checked-in demo was run on 2026-07-28 with five workers and OpenAI API-key
authentication. Durable evidence is stored at:

```text
/home/cc/spatial-real-pilot/benchmark-runs/spatial-10-case-demo-20260728
```

Results:

- 10/10 jobs completed;
- 10/10 answers were scored;
- 7/10 answers were correct;
- 10/10 native session receipts are complete;
- 10/10 native session and export verifications passed;
- 10/10 submissions were accepted;
- 150 sandbox calls were retained; and
- no failure records were produced.

The following correct same-room result is a useful session-review example. It
contains 16 sandbox calls, including 11 Python commands, and a 38-entry native
session:

```sh
uv run --no-sync pi-baseline session view \
  /home/cc/spatial-real-pilot/benchmark-runs/spatial-10-case-demo-20260728/private/spatial-10-case-demo-20260728/c0dd4fe6c2a4e1e3cd13d3662ad767b05f8b08d47e298b9a47a92969affbfe9b/attempt-1/private/pi-e87aba7223926977da0c6df578fa27d163179c9c998666eb4/visualization-forbidden
```

Viewer verification returned HTTP 200 for both the application shell and its
immutable `session.json` document. The submitted answer was `true`, matching
the private expected value.
