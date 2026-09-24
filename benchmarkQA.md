### Templates

Reusable question families for different scenes and simulator environments. Keep questions short and answer-focused; choose scene-specific references separately.

1. Object Location - Identify which room or region contains a named object. Answer: single-choice letter.
2. Object Existence - Determine whether a specified object exists in the environment. Answer: yes/no.
3. Object Counting - Count instances of a specified object class. Answer: exact count.
4. Universal Relation - Determine whether every object of one class has a stated relationship to another object. Answer: yes/no.
5. Room Counting - Count the rooms in the environment. Answer: exact count.
6. Room Size - Ask for either the largest room's name or its approximate area, with one answer per question. Answer: single-choice letter or numerical estimate.
7. Object Dimensions - Estimate an object's height, length, width, diagonal, or footprint area. Answer: one numerical estimate.
8. Boundary Perimeter - Estimate the perimeter of an environment or room. Answer: numerical estimate.
9. Doorway Counting - Count doorways connecting specified spaces. Answer: exact count.
10. Passage Clearance - Estimate the largest robot radius that fits through specified doorways. For a 2D width-only question, the reference is half the narrowest clear width. Answer: numerical estimate.
11. Object Displacement - Estimate straight-line distance or displacement between two objects. Specify the distance measure or coordinate component so a scalar question requires one number. Answer: numerical estimate.
12. Object State - Identify an object's state in the static scene. Answer: single-choice letter.
13. Room Connectivity - Count the minimum doorway crossings between two rooms. Answer: exact count.
14. Clearance Counterfactual - Determine whether a hypothetical object-state change would affect passage through the scene. Answer: yes/no.
15. Wall Separation - Identify objects on opposite sides of the same wall. Answer: single-choice letter.
16. Spatial Ordering - Select the closest object or rank objects from nearest to farthest under a specified distance measure. Answer: single-choice letter or ordered letter sequence.
17. Coverage Confidence - Determine whether a specified set of observations is sufficient to answer a question. Answer: yes/no.

### Scoring functions

Use the existing functions in `dimos/evals/scorers.py`. Parsing model text and scoring typed values are separate. No JSON response is required.

| Answer type | Requested response | Parsing and scoring |
|---|---|---|
| Single choice | One option letter, e.g. `B` | `exact(expected_letter, answer.strip().upper())` |
| Boolean | `yes` or `no` | `exact(expected_yes_no, yes_no(answer))` |
| Exact count | One number | `exact(reference_count, first_number(answer))` |
| Numerical estimate | One number in the requested units | `numeric(reference, first_number(answer), tolerance=t, band=b)` |
| Ranking | Every option letter once, ordered; e.g. `BCA` or `B, C, A` | `rank_order(expected_order, ranking(answer))` |

#### Single choice

- List labeled options separately from the question and request only one letter.
- Correct normalized letter scores **1**; incorrect or invalid replies score **0**.
- Full option strings, explanations, JSON wrappers, and multiple selections are invalid under this response contract.
- Use plausible alternatives of the same kind and exactly one correct option. Use two or three choices when natural rather than padding to four.
- Balance correct-answer positions across a suite. Keep option order identical across harnesses; reproducible shuffling must also update the reference-letter mapping.

#### Boolean

- Request `yes` or `no`; exact match after `yes_no()` scores **1**, otherwise **0**.
- The existing parser normalizes case and accepts replies beginning with yes/no, including explanations. It does not parse `true`/`false`.
- Catch parser `ValueError` in the case grader and return **0** for unparseable answers.

#### Exact count

- Compare the parsed number by value: `4`, `4.0`, and prose whose first number is 4 are equivalent.
- A fractional value such as 4.5 does not match a reference count of 4.
- Discrete object, room, and doorway counts score **1 or 0**, without an estimation band.
- Catch parser `ValueError` and return **0** for unparseable answers.

#### Numerical estimate

For reference `r`, parsed answer `v`, tolerance `t`, and outer error band `b`, require `0 <= t < b` and let `e = abs(v - r)`:

- **Full credit:** `1` when `e <= t`.
- **Partial credit:** `(b - e) / (b - t)` when `t < e < b`.
- **Zero credit:** `0` when `e >= b`.

Example: reference **400**, `tolerance=10`, `band=50`:

| Answer | Score |
|---|---:|
| 390–410 | 1 |
| 370 or 430 | 0.5 |
| 350 or 450, or farther away | 0 |

Choose references and bands independently for each environment and measurement. Full-credit endpoints are included; outer endpoints score zero. `numeric()` handles non-finite observations with zero credit and uses a small floating-point rounding allowance at boundaries.

`numeric()` compares general signed values; physical validity restrictions, if needed, belong in the case grader. Units must match the question; there is no automatic unit conversion.

`first_number()` extracts the first ordinary decimal number rather than validating the whole response. Request decimal notation without scientific notation or thousands separators. Catch parser `ValueError` and return zero for unparseable answers.

#### Ranking

- Request a complete ordered sequence of labels, nearest to farthest or another explicitly stated order.
- `ranking()` normalizes case and removes commas and whitespace. `rank_order()` requires a complete permutation of the expected labels.
- Missing, repeated, extra, or unknown labels score **0**. JSON lists and explanatory prose are not accepted.
- Score the fraction of correctly ordered pairs. For `N` objects there are `N × (N - 1) / 2` pairs.

Generic example with expected order **BCA**:

| Answer | Score |
|---|---:|
| BCA | 1 |
| BAC or CBA | 2/3 |
| ABC or CAB | 1/3 |
| ACB | 0 |

Order matters: this is ranking, not multi-select. Validate that the reference has a unique order; revise ambiguous or tied cases before assigning a strict ranking reference.

#### Shared conventions

- Keep reference values, source annotations, and scoring parameters author-side. Send only the question, options, and answer-format instructions to evaluated agents.
- Establish scene-specific measurement conventions: coordinate frame, object centers versus surfaces, room boundaries, 2D versus 3D distance, and robot footprint where relevant.
- Use templates only where the environment supports observable, defensible answers. Counterfactuals need suitable state geometry; coverage questions need a defined observation premise.
- Mark provisional references for review rather than inventing answers for unresolved cases.
- `EvalCase.threshold=1.0` means only full-credit answers pass. Partial scores remain available for aggregate comparison.
- No multi-select parser or scorer is specified here; add one if a future question genuinely requires an unordered set of choices.

### Other questions
