### Templates

1. Object Location - Identify which room contains a named object.
2. Object Existence - Determine whether a specified object exists in the house.
3. Object Counting - Count instances of a specified object class.
4. Universal Relation - Determine whether every object of one class has a stated relationship to another object.
5. Room Counting - Count the rooms in the house.
6. Room Size - Ask for either the largest room's name or its approximate area, with one answer per question.
7. Object Dimensions - Estimate an object's height, length, width, or footprint area.
8. Boundary Perimeter - Estimate the perimeter of the house or a room.
9. Doorway Counting - Count doorways connecting specified spaces.
10. Passage Clearance - Estimate the largest robot radius that fits through specified doorways.
11. Object Displacement - Estimate straight-line distance or displacement between two objects.
12. Object State - Identify an object's state in the static scene.
13. Room Connectivity - Count the minimum doorway crossings between two rooms.
14. Clearance Counterfactual - Determine whether a hypothetical object-state change would affect passage through the scene.
15. Wall Separation - Identify objects on opposite sides of the same wall.
16. Spatial Ordering - Select the closest object under a specified distance measure; ask straight-line and travel-distance comparisons separately.
17. Coverage Confidence - Determine whether a specified set of observations is sufficient to answer a question.

### Questions apartment

1. [Object Location] Which room contains the refrigerator?
   - **Options:** A) Bedroom; B) Kitchen; C) Bathroom; D) Living room.
   - **Answer type:** Single-choice letter. Return only A, B, C, or D.
   - **Scoring:** `exact("B", answer.strip().upper())`: **B** = **1**, otherwise **0**.

2. [Object Location] Which room contains the work desk?
   - **Options:** A) Kitchen; B) Bathroom; C) Living room; D) Bedroom.
   - **Answer type:** Single-choice letter. Return only A, B, C, or D.
   - **Scoring:** `exact("D", answer.strip().upper())`: **D** = **1**, otherwise **0**.

3. [Object Existence] Does the house contain a bathtub?
   - **Answer type:** Yes/no; return only `yes` or `no`.
   - **Scoring:** `exact("yes", yes_no(answer))`: correct = **1**, incorrect or unparseable = **0**.

4. [Object Existence] Does the house contain a washing machine?
   - **Answer type:** Yes/no; return only `yes` or `no`.
   - **Scoring:** After validating absence, `exact("no", yes_no(answer))`: correct = **1**, otherwise **0**. **Reference remains provisional.**

5. [Object Counting] How many dining chairs are in the house?
   - **Answer type:** Plain count, e.g. `4`; parsed with `first_number()` and compared by value.
   - **Scoring:** Integer count **4** = **1**, any other count = **0**. No partial credit for these small, discrete counts.

6. [Object Counting] How many bedside tables are in the house?
   - **Answer type:** Plain count, e.g. `2`; parsed with `first_number()` and compared by value.
   - **Scoring:** Integer count **2** = **1**, otherwise **0**.

7. [Object Counting] How many wall-mounted kitchen cabinets are in the house?
   - **Answer type:** Plain count, e.g. `3`; parsed with `first_number()` and compared by value.
   - **Scoring:** Integer count **3** = **1**, otherwise **0**.

8. [Universal Relation] Does every work desk in the house have a laptop on it?
   - **Answer type:** Yes/no; return only `yes` or `no`.
   - **Scoring:** After validating the support relationship, `exact("yes", yes_no(answer))`: correct = **1**, otherwise **0**. **Reference remains provisional.**

9. [Room Counting] How many rooms are in the house?
   - **Answer type:** Plain count, e.g. `4`; parsed with `first_number()` and compared by value.
   - **Scoring:** Integer count **4** = **1**, otherwise **0**, once the room-count convention is approved. No partial credit.

10. [Room Size] What is the approximate area of the largest room, in square meters?
    - **Answer type:** Plain numerical estimate in m², parsed with `first_number()`.
    - **Scoring:** `numeric(37.8, first_number(answer), tolerance=2.5, band=8.0)`. **1** for **35.3–40.3 m²**; linear partial credit out to **29.8/45.8 m²**; **0** at or beyond those outer limits. Validate the floor polygon before finalizing the reference.

11. [Object Dimensions] What is the approximate height of the refrigerator, in meters?
    - **Answer type:** Plain numerical estimate in meters, parsed with `first_number()`.
    - **Scoring:** `numeric(1.80, first_number(answer), tolerance=0.10, band=0.40)`. **1** for **1.70–1.90 m**; linear partial credit out to **1.40/2.20 m**; **0** at or beyond those outer limits.

12. [Object Dimensions] What are the approximate length and width of the dining table, in meters?
    - **Answer type:** Two positive numbers in meters separated by a comma: `length, width`, e.g. `2.2, 1.1`.
    - **Scoring:** Parse exactly two comma-separated floats and normalize the larger to length. Average **0.5 × length score + 0.5 × width score**. Length reference **2.20 m**, `tolerance=0.10`, `band=0.40`: full credit **2.10–2.30 m**, zero at or outside **1.80–2.60 m**. Width reference **1.10 m**, `tolerance=0.05`, `band=0.25`: full credit **1.05–1.15 m**, zero at or outside **0.85–1.35 m**. Intermediate errors receive linear partial credit. Swapping the dimensions does not lose points; missing, extra, non-finite, or nonpositive values score zero.

13. [Object Dimensions] What is the approximate footprint area of the bed frame, in square meters?
    - **Answer type:** Plain numerical estimate in m², parsed with `first_number()`.
    - **Scoring:** `numeric(3.52, first_number(answer), tolerance=0.25, band=1.0)`. **1** for **3.27–3.77 m²**; linear partial credit out to **2.52/4.52 m²**; **0** at or beyond those outer limits.

14. [Boundary Perimeter] What is the approximate perimeter of the house, in meters?
    - **Answer type:** Plain numerical estimate in meters, parsed with `first_number()`.
    - **Scoring:** `numeric(44.0, first_number(answer), tolerance=1.0, band=5.0)`. **1** for **43–45 m**, covering both the slab outline and outer-wall envelope. Linear partial credit out to **39/49 m**; **0** at or beyond those outer limits. Confirm the house boundary before finalizing.

15. [Doorway Counting] How many doors connect the house to the yard?
    - **Answer type:** Plain count, e.g. `2`; parsed with `first_number()` and compared by value.
    - **Scoring:** Integer count **2** = **1**, otherwise **0**, after confirming the two exterior openings correspond to the intended door count.

16. [Passage Clearance] What is the largest robot radius that can fit through all the doorways, in meters?
    - **Answer type:** Plain numerical estimate in meters, parsed with `first_number()`.
    - **Scoring:** **Pending actual-clearance ground truth.** For validated reference radius `r`, propose `numeric(r, first_number(answer), tolerance=0.025, band=0.10)`: full credit within **±2.5 cm**, linear partial credit out to **±10 cm**, then **0**. Do not substitute the structural upper bound for an unvalidated actual-clearance answer.

17. [Object Displacement] What is the approximate straight-line distance between the refrigerator and television, in meters?
    - **Answer type:** Plain numerical estimate in meters, parsed with `first_number()`.
    - **Scoring:** `numeric(8.37, first_number(answer), tolerance=0.30, band=1.50)`. **1** for **8.07–8.67 m**; linear partial credit out to **6.87/9.87 m**; **0** at or beyond those outer limits. Validate the object-center convention before finalizing.

18. [Object State] Is the refrigerator open or closed?
    - **Options:** A) Closed; B) Open.
    - **Answer type:** Single-choice letter. Return only A or B.
    - **Scoring:** `exact("A", answer.strip().upper())`: **A** = **1**, otherwise **0**.

19. [Room Connectivity] What is the minimum number of doorway crossings between the kitchen and bathroom without leaving the house?
    - **Answer type:** Plain count, e.g. `3`; parsed with `first_number()` and compared by value.
    - **Scoring:** Provisional integer count **3** = **1**, otherwise **0**. Finalize only after checking the room-connectivity reference; no partial credit for crossing count.

20. [Clearance Counterfactual] Would opening the refrigerator change whether a robot of radius 0.25 m can pass from the kitchen doorway to the sink?
    - **Answer type:** Yes/no; return only `yes` or `no`.
    - **Scoring:** **Pending geometric comparison.** Once expected answer `b` is established as `"yes"` or `"no"`, use `exact(b, yes_no(answer))`: correct = **1**, otherwise **0**. Compare whether a route exists in each state, not merely whether its width or length changes.

21. [Wall Separation] Which pair of objects is on opposite sides of the same wall?
    - **Options:** A) Sofa and television; B) Work desk and bed; C) Refrigerator and work desk; D) Bathtub and toilet.
    - **Answer type:** Single-choice letter. Return only A, B, C, or D.
    - **Scoring:** `exact("C", answer.strip().upper())`: **C** = **1**, otherwise **0**, after validating the pair geometry. A list of several options is invalid; this is not multi-select.

22. [Spatial Ordering] Which object is closest to the sofa by straight-line distance?
    - **Options:** A) Refrigerator; B) Dining table; C) Work desk.
    - **Answer type:** Single-choice letter. Return only A, B, or C.
    - **Scoring:** `exact("B", answer.strip().upper())`: **B** = **1**, otherwise **0**. Reference remains provisional until the center-based distance calculation is validated. No partial credit for selecting the second-closest object.

23. [Spatial Ordering] Which object is closest to the sofa by collision-free travel distance for a robot of radius 0.25 m?
    - **Options:** A) Refrigerator; B) Dining table; C) Work desk.
    - **Answer type:** Single-choice letter. Return only A, B, or C.
    - **Scoring:** **Pending path-based ground truth.** Map the validated closest object to its option letter, then use `exact(expected_letter, answer.strip().upper())`: correct = **1**, otherwise **0**. Validate a unique winner before finalizing this single-choice case; if there is a tie, revise the options or explicitly use multi-select. Do not assume Q22's answer applies.

24. [Coverage Confidence] Is observing only the living room and kitchen sufficient to determine how many bedside tables are in the house?
    - **Answer type:** Yes/no; return only `yes` or `no`.
    - **Scoring:** Provisional `exact("no", yes_no(answer))`: correct = **1**, otherwise **0**. Requires validation of the observation premise described in the reference notes.

### Scoring conventions

- **All references and tolerance bands above are draft for review.** The 21 cases with draft answers are implemented in `dimos/evals/suites/dimsim_apartment_qa.py`; Q16, Q20, and Q23 remain pending. Keep reference values author-side; send only questions, answer-format instructions, and options to the agent. No JSON response is required.
- **Parsing and scoring are separate:** Reuse the existing `first_number()` and `yes_no()` parsers. Suite-local `_parsed()` catches parser `ValueError` and assigns zero; scorers compare the resulting typed values. The global parsers retain their existing behavior.
- **Boolean:** Request `yes` or `no`; use `exact(expected, yes_no(answer))`. The existing parser normalizes case and accepts replies beginning with yes/no, including explanations. A reply of `true`/`false` is not the requested yes/no format and is unparseable by this parser.
- **Exact count:** Request the count and use `exact(reference, first_number(answer))`. `4`, `4.0`, and prose whose first number is 4 are equivalent. A fractional value such as 4.5 does not match 4. Counts get **1 or 0**, not partial credit.
- **Single-choice:** Provide labeled options separately from the question and request only one letter. Trim whitespace and uppercase the reply, then exact-match the expected letter. Full option strings, JSON wrappers, explanations, and multiple selections receive zero under this bare-letter contract. Q1, Q2, Q18, Q21, Q22, and Q23 use this format.
- **Choice design:** Use plausible alternatives of the same kind and exactly one correct option. Use two or three options when natural rather than padding to four. Balance correct-answer positions across the suite after references are validated. Keep option order identical across harnesses; any shuffle must be reproducibly seeded and update the reference-letter mapping. Measurements retain numerical answers and tolerance scoring rather than multiple-choice ranges.
- **Numerical estimate:** Use `numeric(reference, first_number(answer), tolerance=t, band=b)`. Let `e = abs(value - reference)`: score **1** for `e <= t`, **(b - e) / (b - t)** for `t < e < b`, and **0** for `e >= b`. Non-finite observations score zero. A small rounding allowance handles decimal endpoints without consuming a meaningful portion of the band. `numeric()` is a general typed-value scorer, so negative values are not inherently invalid; negative measurements fall outside every active case's accepted band. Units are those requested by the question; there is no unit conversion.
- **Parser limits:** `first_number()` uses the existing decimal-number extraction, not a strict whole-answer validator. Ask for ordinary decimal notation without thousands separators or scientific notation; do not assume the parser understands those formats. Unparseable responses score zero. These parser semantics are shared with existing evals rather than changed globally by this suite.
- **Example:** For reference **400**, `tolerance=10`, `band=50`: **390–410** gets **1**; **370 or 430** gets **0.5**; **350 or 450**, and anything farther away, gets **0**. Partial credit varies continuously with error.
- **Two measurements (Q12):** Parse exactly two comma-separated positive finite numbers, normalize length/width ordering, score each independently, then average. A completely wrong dimension and a fully correct other dimension give **0.5**; both must be within their tolerances for full credit.
- **Multi-select:** None of the current questions requests multiple selections; no multi-select parser or scorer is added.
- **Unvalidated references:** Leave the case ungraded until its reference is approved; do not assign a fabricated expected answer or score all attempts zero. This applies especially to Q16, Q20, and Q23.
- **Pass/fail:** A proposed `EvalCase.threshold=1.0` means only full-credit answers pass. Partial scores remain available for aggregate comparison and diagnosis.

### Other questions

### Ground truths apartment

Author-only draft references for user validation. The questions request answers, not movement or observation procedures. They are independent of dimOS tools and are not executable EvalCases. Keep source data and reference answers separate from the evaluated agent's context.

Sources inspected at checkout commit `c5b78bd9a23344db91a59aabdd7f7db614998ab8`:

- `misc/DimSim/scenes/apartment/index.js`: assembly, structure loading, object manifest loading, and authored spawn.
- `misc/DimSim/scenes/apartment/objects/manifest.json`: 87 placed asset entries, transforms, and selected states.
- `misc/DimSim/scenes/apartment/structure.glb`: named structural meshes, node transforms, and position bounds.
- Selected object-state GLBs: scene-graph bounding extents, including internal node transforms. The measured refrigerator, dining table, and bed have unit placement scale; the table and bed are rotated 90 degrees around the vertical axis, which does not change their length/width or footprint area.

Reference conventions, kept out of the question wording:

- Scene coordinates are **Three.js coordinates**, in meters: X/Z horizontal, Y up. The bridge maps them to ROS coordinates as `(x, y, z) -> (z, x, y)`.
- The authored spawn is `(2, 0.5, 3)`, not the origin. The runner handles the agreed common start and any origin rebasing. Source positions below remain in the authored frame.
- The benchmark scene stays static. State answers refer to the selected states in the manifest; no state-change episodes are required. Question 20 asks about a hypothetical change, not an actual state-changing task.
- Count interior rooms, with the open-plan living/dining area treated as one room and the yard excluded. Count physical object instances, not meshes or alternate state files.
- Room area is floor area between inside wall faces, including furniture-occupied floor. Object dimensions use complete asset bounds; the bed footprint includes the headboard but excludes separate furniture and loose bedding.
- Distance references below use horizontal object-center distance. Placed asset anchors are preliminary center proxies, requiring validation. Doorway-radius calculations assume a circular footprint; actual traversability also depends on door panels, furniture, and height.
- Numerical tolerances are for the user to validate. Source precision below is not required answer precision. If different reasonable interpretations of a short question produce materially different answers, accept the relevant range or clarify that question before scoring.

| Question | Draft ground truth | Evidence and validation needed |
|---|---|---|
| 1 | **Kitchen.** | Refrigerator asset `b500b00d33e638-19c735e91d7`, scene anchor approximately `(-2.639, 0.988, 0.519)`, lies on the kitchen side of the partition. Grade room identification, not navigation. |
| 2 | **Bedroom.** | Work-desk asset `bc6db6f4cbd0a-19c732ef8c8`, scene anchor approximately `(-1.730, 0.465, -0.433)`. Grade room identification, not navigation. |
| 3 | **True.** | One placed bathtub entry: `Freestanding oval soaking bathtub with chrome fl...`. |
| 4 | **Candidate: false; validate visually before use.** | No washing-machine entry in the manifest; there is a dishwasher. Manifest absence alone does not exclude an object embedded in another GLB or the static structure. |
| 5 | **4.** | Four `Dining chair` entries. |
| 6 | **2.** | Two `Bedside table` entries referencing the same asset-state files. |
| 7 | **3.** | Three `kitchen wall cabinet` entries, distinct from three base cabinets. |
| 8 | **Candidate: true.** | One work desk and one laptop. Horizontal anchors are approximately `(-1.730, -0.433)` and `(-1.700, -0.410)`; the laptop is above the desktop. Validate the rendered support relationship. With only one qualifying desk this is a weak test of universal reasoning. |
| 9 | **4.** | Living/dining, kitchen, bedroom, bathroom. Main partition near scene `z=0`, kitchen partition at `x=-2` on the positive-Z side, bathroom partition at `x=1` on the negative-Z side. The #3879 mapping-suite reference also uses four. Validate semantic room boundaries. |
| 10 | **Approximately 37.8 m².** | The largest room is living/dining, but the requested answer is only its area. Approximate inside-face bounds: scene X `[-1.925, 5.9]`, Z `[0.075, 4.9]`; `7.825 × 4.825 = 37.755625`. A small offset of the main-left wall makes this approximate; validate the final floor polygon. Wall-centerline dimensions give 40 m², relevant when setting a reasonable estimation tolerance. |
| 11 | **Approximately 1.801 m.** | Closed refrigerator GLB vertical extent `1.80074978 m`; placement scale is one. Appliance height rather than its center elevation. |
| 12 | **Approximately 2.20 m × 1.10 m.** | Selected dining-table GLB horizontal extents; placement scale is one. |
| 13 | **Approximately 3.52 m².** | Complete bed-frame GLB horizontal extents approximately `1.60 × 2.20 m`, including the headboard. Rectangular footprint, not mattress area or mesh surface area. |
| 14 | **Approximately 44 m using the floor-slab outline.** | `apartment-floor` spans scene X `[-6, 6]`, Z `[-5, 5]`: `2 × (12 + 10)`. The outside faces of the 0.2 m thick exterior walls give an approximate rectangular envelope of `12.2 × 10.2 m`, or 44.8 m perimeter. Validate the intended boundary and tolerance for the broader house-perimeter wording; exclude the yard. |
| 15 | **2 exterior openings; validate the rendered door interpretation.** | Openings under `wall-south-header-entrance` and `wall-south-header-sliding` connect the house to the yard. The earlier total of five also counted three interior openings, which this question no longer asks about. |
| 16 | **Structural upper bound: approximately 0.50 m radius; actual passable radius pending validation.** | Structural openings are approximately `1, 1, 1, 1, 2 m` wide, so half the minimum width is `0.50 m`. Positive clearance requires a smaller radius. The short question no longer excludes panels or furniture, so check the actual bottleneck before using this as a final ground truth. A tolerance alone does not establish actual passability. |
| 17 | **Approximately 8.37 m horizontally, using asset anchors as center proxies.** | Refrigerator-to-TV scene-plane displacement approximately `(ΔX, ΔZ) = (+7.139, +4.361) m`; distance `8.36548 m`. ROS horizontal displacement is `(+4.361, +7.139) m`. Validate geometric centers. Height difference is approximately 0.659 m, so a 3D interpretation gives approximately 8.39 m; account for this when setting tolerance. |
| 18 | **Closed.** | Refrigerator `currentStateId=state-default`, state name `closed`. |
| 19 | **Candidate: 3 crossings.** | The structural room graph suggests kitchen → living/dining → bedroom → bathroom. Validate the passage connectivity against the full geometry; this is a topological count, not a measured route length. |
| 20 | **Pending geometric validation.** | Compare the closed and open refrigerator geometry with the sink approach and a 0.25 m circular footprint. Do not infer the answer from the existence of an open-state asset alone. |
| 21 | **Candidate: C, refrigerator and work desk.** | The refrigerator is at scene Z approximately `+0.519` and the desk at `-0.433`, on opposite sides of the main-left partition near `z=0`. Their connecting line crosses that partition. The other pairs occupy the same respective room. Validate against the rendered scene. This is wall separation, not viewpoint-dependent occlusion. |
| 22 | **Candidate: dining table.** | Manifest anchors put the table approximately 5.49 m from the sofa, versus approximately 6.29 m for the desk and 7.04 m for the refrigerator in the horizontal plane. Validate geometric centers before finalizing. |
| 23 | **Pending path computation.** | Requires shortest collision-free paths and consistent reachable approach points around the sofa and each candidate object; object centers themselves can lie inside obstacles. Do not reuse the straight-line answer without checking. |
| 24 | **Candidate: false.** | The bedside tables are in the bedroom. This tests evidence sufficiency, not the total count itself. Validate the restricted observation set: it must not already reveal both tables through a doorway or provide an earlier complete-house view. |

All references remain draft for user validation. Questions with pending geometry or observation checks should not receive fixed-answer scorers until those checks are complete. Expected answer types are single-choice letters, booleans, counts, or numerical measurements; none requires a descriptive essay or evidence narrative.
