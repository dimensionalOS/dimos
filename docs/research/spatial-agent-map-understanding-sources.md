# Spatial agent map understanding: source review and experiment implications

Date: 2026-07-23

## Scope

This note reviews primary sources relevant to a narrow question: **what
representation lets an agent reliably answer spatial questions and place
actionable points or poses from robot maps?** It treats map comprehension,
coordinate grounding, frame transforms, semantic grounding, and navigation
feasibility as separate capabilities. Claims reported by sources are separated
from proposed experiments below.

## What the existing DimOS work established

[DimOS issue #1913](https://github.com/dimensionalOS/dimos/issues/1913) asks how
an occupancy grid or point cloud returned by memory should be presented to an
LLM. It proposes two families of evaluation:

- comprehension: room count and size, largest-room location, and space type;
- point placement: exploration targets, points in largest/smallest rooms, and
  hallway markers.

The issue explicitly warns that models may guess and therefore calls for many
specific questions. It links
[PR #822](https://github.com/dimensionalOS/dimos/pull/822) as an earlier,
unsuccessful attempt to render an occupancy grid as a VLM image.

The PR is useful evidence, but it is not a decisive rejection of rendered maps:

- It rendered the grid as RGB with a robot-pose overlay and asked both
  comprehension and placement questions. Reported acceptance floors were 0.70
  for comprehension and 0.25 for point placement, from only two floorplans plus
  variations.
- Its author reported that body-up normalization helped robot-relative
  questions, orientation questions were harder, noise hurt orientation, and
  placement was sensitive to wording, image scale, and aspect ratio.
- Review identified a likely transform/rendering problem and a visibly
  incorrect semantic answer
  ([critique](https://github.com/dimensionalOS/dimos/pull/822#issuecomment-3662470171)).
  The author then fixed pixel-to-world conversion and reported that changing
  the phrase to include “long table” changed the answer
  ([response](https://github.com/dimensionalOS/dimos/pull/822#issuecomment-3665006521)).
  This is evidence of a confounded pipeline and prompt sensitivity, not robust
  map understanding.
- The PR was ultimately closed because it was old and had not been rebased
  ([closure comment](https://github.com/dimensionalOS/dimos/pull/822#issuecomment-3940123113)),
  not because a controlled representation comparison had settled the question.

The unresolved variables are therefore: raw versus rendered versus structured
maps; local/body-frame versus global/world-frame context; pixel versus
normalized versus metric outputs; and geometry-only versus semantically
augmented inputs.

## Findings from primary sources

### Coordinate grounding should be measured as its own primitive

[SeeClick and ScreenSpot](https://arxiv.org/abs/2401.10935) define GUI
grounding as locating a screen element from an instruction. ScreenSpot contains
more than 600 screenshots and 1,200 instructions across mobile, desktop, and
web interfaces, with target bounding boxes. Its click metric counts a prediction
as correct when the predicted point lies inside the target box. The authors
found that general-purpose vision-language models struggled with this grounding
task and that grounding improvements correlated with downstream GUI-agent
performance.

This supports isolating “place a cursor on the requested region” from semantic
map reasoning and pixel-to-world conversion. It does **not** show that GUI
grounding transfers to occupancy maps.

### A screenshot and a structure expose different information

[VisualWebArena](https://aclanthology.org/2024.acl-long.50/) evaluated agents
using accessibility trees, screenshot plus accessibility tree, and screenshots
annotated with unique IDs and boxes (“set of marks”). In the paper's GPT-4V
baselines, screenshot + captions + set of marks scored 16.37%, while screenshot
+ captions + accessibility tree scored 15.05%; human success was 88.70%.
Because the formats and action interfaces differ, this is not a clean causal
comparison. It does demonstrate an established evaluation pattern: hold tasks
fixed while varying pixels, structured state, and visual markers.

[OSWorld](https://arxiv.org/abs/2404.07972) likewise implements screenshots,
XML-formatted accessibility trees, and terminal output as distinct observation
channels. These benchmarks make a structured “map accessibility tree” a
reasonable experimental condition, not an assumption that structure will win.

### Local-to-global frame alignment is independently difficult

[REMAP: The Representational Mapping
Benchmark](https://openreview.net/pdf?id=4hQxcX62TQ) isolates cross-view
geometric correspondence: the model must align local egocentric views with a
global allocentric overhead map under viewpoint changes, with semantic landmark
cues deliberately minimized. This directly motivates separating:

1. understanding free/occupied geometry in one frame;
2. locating and orienting the robot in that frame; and
3. transforming a requested point or pose between egocentric and allocentric
   frames.

The DimOS PR observation that “body-up” helped is a hypothesis to retest, not a
general result. A local body-frame crop may simplify immediate action mapping
while discarding global topology; a global map may preserve topology while
adding localization and rotation burden.

### RGB semantics and occupancy geometry are complementary

[VLFM](https://arxiv.org/abs/2312.03275) constructs occupancy maps from depth to
identify free-space frontiers, while a vision-language model scores RGB
observations against an object-language goal and writes those scores into a
spatial value map. The resulting system uses geometry for navigable exploration
and RGB for semantic guidance, and the paper reports evaluation in Gibson,
HM3D, and Matterport3D plus deployment on a real Spot robot.

This is evidence for testing semantic projection as a separate representation:
an occupancy map alone should not be expected to distinguish a couch from a
table, but an object detection, bearing/range observation, or projected semantic
footprint can be fused with the map.

### Markers can help or distract; do not assume arrows solve grounding

[Anthropic's 2026 robotics
experiments](https://www.anthropic.com/research/claude-plays-robotics) compare
raw visual input with text scene descriptions and additional overlays. The
authors report that stronger models retained useful detail from raw images that
was lost in text descriptions, while most extra visual aids were neutral or
slightly harmful. A crosshair sometimes helped a Go2 agent align in a hallway,
but in another trial the model misjudged a trash can relative to the crosshair
and collided with it.

This first-party report is not an occupancy-map benchmark, but it is a concrete
warning that arrows, cursors, compasses, depth overlays, and verbal summaries
must each be ablated and checked against physical outcomes.

### A 2D game demonstrates persistence, not precise map understanding

In [Claude plays
Pokémon](https://www.anthropic.com/news/visible-extended-thinking), Anthropic
gave an agent screen pixels, memory, and button-press tools over tens of
thousands of interactions. The demonstration establishes that a pixel-based
agent can make long-horizon progress in a top-down game. It does not isolate
coordinate accuracy, frame transforms, collision reasoning, or sample
efficiency. A game environment is useful as a controllable testbed only if those
capabilities receive explicit, programmatic scoring.

## Proposed experiments (not source claims)

### 1. Establish non-map grounding floors

Use generated images with six randomized colored rectangles and ask for one
target point. Vary resolution, aspect ratio, target size, color, wording, and
layout. Score:

- point-in-region accuracy;
- normalized coordinate error;
- invariance under resize, crop, rotation, and translation;
- calibration, including “the requested target is absent.”

Compare pixel coordinates, normalized `[0, 1]` coordinates, a numbered grid,
and set-of-marks IDs. This determines whether a failure is cursor grounding
before adding robot-map complexity.

### 2. Run a factorial map-representation benchmark

Render the **same underlying scene and question** in paired conditions:

| Factor | Conditions |
|---|---|
| Geometry | raw point cloud/grid; decoded numeric cells; PNG/SVG; ASCII/RLE; polygons/room graph |
| Extent | global; 5 m local; 10 m local; global + local |
| Frame | north/world-up; body-up; randomized global rotation |
| Pose marker | none; point; arrow; footprint; footprint + axes |
| Output | Boolean/count; region ID; pixel/normalized point; metric point; metric pose `(x, y, yaw)` |
| Agent mode | one-shot; fixed map tools; unrestricted coding agent |

Use repeated scenes so paired statistical tests can attribute differences to a
representation rather than question difficulty. Do not tune prompts separately
per condition unless prompt tuning is itself the independent variable.

### 3. Decompose semantics from geometry

For tasks such as “go to the other side of the couch” or “sit beside me,” use
four paired inputs:

1. occupancy only;
2. RGB only;
3. occupancy + oracle object class, bearing, and range;
4. occupancy + projected object footprint and orientation.

Ask first for object grounding, then a target point, then target yaw, then
collision/path feasibility. Report failures at each boundary. The occupancy-only
condition is a negative control for object identity.

### 4. Make the evaluation adversarial against guessing

- Include no-map and heavily downsampled-map baselines.
- Create counterfactual pairs differing by one doorway, one obstacle, robot
  heading, or object projection while keeping wording constant.
- Balance yes/no labels and count distributions.
- Include false-premise questions, such as asking what is down a hallway when
  the robot faces a wall.
- Randomize map rotation, origin, resolution, crop, and irrelevant clutter.
- Hold out complete physical spaces, not merely questions from seen maps.
- Score final coordinates by geometry: free-space membership, footprint
  collision, reachability, path length, and yaw tolerance.

### 5. Move to real data before autoresearch

After synthetic capability isolation, replay a fixed real Go2 recording such as
the Hong Kong office global map. Build human-verified annotations for robot
pose, rooms, doors, traversable regions, and—when RGB/object questions are
included—object observations and map projections. Compare synthetic-to-real
degradation. Only after the failure categories and metrics are stable should an
autoresearch loop optimize a representation or helper tool.

## Analyses needed to answer the current team questions

For every shared question/agent-answer sample, retain the exact input artifact,
prompt, answer, submitted coordinate/pose, tool transcript, and ground truth.
Then report:

1. accuracy by capability (comprehension, point placement, frame transform,
   semantics, collision, topology), not only aggregate run completion;
2. paired accuracy by representation, crop, frame, and marker condition;
3. coordinate-error distributions and physically executable success, not
   qualitative screenshots;
4. no-input and degraded-input baselines to estimate guessing and shortcuts;
5. wording consistency on paraphrase sets;
6. error attribution at the first failed stage: decode, perceive, localize,
   transform, reason, or convert coordinates;
7. separate one-shot results from coding-agent results, including runtime,
   command count, and whether the agent wrote a task-specific algorithm.

That analysis can answer “does the agent understand the map?” more precisely:
it identifies **which representation, spatial scale, coordinate frame, output
contract, and amount of computation support which operational capability**.
