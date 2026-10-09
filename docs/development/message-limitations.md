# Native CDR proposal: accepted limitations

The proposal directly uses rosbags 0.11.0 for Python, upstream ROSIDL/FastRTPS
with Fast CDR 2.4.0 for C++, and ros2msg 0.5.3 with re_cdr 0.1.0 for Rust.
On 2026-10-09 the user provisionally accepted the limitations below as non-blockers
for continuing this proposal. This is a scope decision, not evidence that malformed
input is safe or correctly decoded. No upstream patch or owned field codec was
added to conceal these behaviors.

Use schema-matched CDR from trusted producers and recordings. The Python decoder
is **not a validation boundary for hostile or malformed input**. A trusted producer
can still have bugs; round-trip tests and transport authentication do not establish
parser safety. No denial-of-service or corruption of valid payloads was demonstrated
by the small cases below; malformed inputs can nevertheless become plausible,
unintended values.

## The original 13 strict acceptance failures

The assertions remain in `dimos/message_codegen/test_python_source.py`. Only the
exact failing parameters carry `xfail(strict=True)` with these identifiers. An
unexpected pass fails the check and requires reviewing the pin and limitation.
Run the original assertions without expected-failure handling using:

```sh
python -m pytest --noconftest -o addopts='' --runxfail \
  dimos/message_codegen/test_python_source.py
```

With the accepted pin, this strict run has **8 passed, 13 failed**; the ordinary
run has **8 passed, 13 xfailed, 0 failed**. This distinction must remain visible in
validation reports. It is not an all-correct decoder result.

### CDR-L01

**Exact-consumption policy: 3 cases.** A valid `Bool` followed by one, two or three
zero bytes is accepted, and those bytes disappear on re-encoding. Our original
rule required consuming the entire payload. Rosbags explicitly tolerates up to
three trailing bytes; framing and padding policies vary, so these tests do not
prove every padded sample is invalid CDR. The decoded boolean is unchanged.
Four extra bytes are still rejected in the ordinary, non-optimized test run.

### CDR-L02

**String wire validity: 2 cases, little and big endian.** A declared string length
of zero, with no terminator, becomes an empty string. Ordinary CDR string lengths
include the NUL terminator. Re-encoding creates length one plus NUL. Some native
libraries also tolerate a zero-length null-string form; this tolerance does not
satisfy the original assertion.

### CDR-L03

**String wire validity: 2 cases, little and big endian.** Length two with payload
`ab` and no NUL decodes as `a`. The decoder discards the final byte without
checking that it is a terminator. This is demonstrated truncation of malformed
input, not demonstrated corruption of a valid encoded string.

### CDR-L04

**String wire validity: 2 cases, little and big endian.** Declared length
`0xffffffff` with only one NUL byte becomes an empty string. The upstream decoder
reads that length as signed `-1`; its returned body position is three although
five bytes were supplied. This demonstrates malformed/truncated-input acceptance,
not a demonstrated four-gigabyte allocation or denial-of-service.

### CDR-L05

**Representation mismatch: 1 case.** Header `00 03 00 00` followed by byte `01`
is accepted as `Bool(True)`. Identifier `0003` means parameter-list CDR little
endian; it is not an unknown identifier, but this body is not a valid parameter
list and the proposal expects plain CDR (`0000` or `0001`). Rosbags interprets it
as plain little-endian CDR instead of rejecting the representation.

### CDR-L06

**Boolean wire validity: 1 original case.** Boolean byte `02` becomes `True`, and
re-encoding changes it to `01`. The invalid value is normalized silently.
The separate existing array regression in `test_definitions.py` exercises the
same limitation in both byte orders: **2 additional explicit xfails**, not two
additional categories. Its valid padding and NaN checks remain ordinary tests.

### CDR-L07

**Bounded-schema support: 1 case.** A valid `shape_msgs/msg/SolidPrimitive` with
three dimensions cannot encode/decode through the Python registry: it raises
`NotImplementedError`. This is a functionality gap. Valid bounded messages do
**not** currently work merely because the types can be constructed.

### CDR-L08

**Bounded-schema error contract: 1 case.** The same type with four dimensions is
also rejected as unsupported, rather than with the assertion's expected bound
validation `ValueError`. It fails closed; this case does not demonstrate an
oversized value being accepted. No bound is silently truncated or removed.

### CDR-L09

The original bounded `demo_msgs/msg/Telemetry` generation is retained as two
strict expected failures (C++ and Rust) in `test_native_libraries.py`. Its `.msg`
source is unchanged. These are additional manifestations of unsupported bounded
strings, not part of the original 13 Python decoder assertions. The built-in
catalog also has two endian-specific `CDR-L07` xfails for `SolidPrimitive`.

## Other native API and schema limits

Python encode/decode rejects every closure containing a bounded string or bounded
sequence. C++/Rust generation rejects bounded-string closures; Rust wide-string
mapping is also unvalidated and rejected. These failures remain errors at the
public boundary. The legacy `demo_msgs/msg/Telemetry` fixture contains both a
bounded string and bounded sequence. Its full three-language/default conformance
and derived viewer/package demos cannot be claimed as passing; its original
schema and assertions remain available for later library work.

Python/Rust construction requires explicit fields. Python native classes do not
apply `.msg` defaults; Rust's upstream floating-default generation is disabled.
These are separate from the 13 wire/bounds cases. Native NumPy arrays and nested
values retain their library ownership/equality behavior; helpers supply explicit
copy/read-only-view contracts. Former pybind container methods are not the native
value API. Rosbags initializes Python classes and codecs in memory from installed
schemas; Python import does not compile native code or download dependencies.
Use the single frozen registry within a runtime process. Independent rosbags
reference stores replace its process-global `usertypes` module and can invalidate
ordinary pickle identities; run independent reference decoders in separate
processes. Oracle tests restore that global module after each case.

C++ distribution is source-only with explicit, on-demand native preparation.
Prebuilt SDK products are out of scope. M20 SDK access, host multicast, Foxglove
sign-in and human visual acceptance are separate limits, not covered by these
expected failures. No benchmark acceptance is implied.

## Source references and future changes

- [Rosbags type system](https://ternaris.gitlab.io/rosbags/topics/typesys.html)
- [Rosbags decoder implementation](https://gitlab.com/ternaris/rosbags/-/blob/v0.11.0/src/rosbags/typesys/store.py)
- [OMG CDR string encoding](https://www.omg.org/cgi-bin/doc?formal%2F01-09-52.pdf=)
- [OMG representation identifiers](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF)

A future upstream version, finite boundary checks or a narrower contract requires
reviewing each expected failure explicitly. Do not broad-skip the suite, silently
relax its assertions, or add a new field codec to make the status green.
