# Embodied benchmark goal representations: primary-source review

Date: 2026-07-29

## Scope

This note reviews how established manipulation, navigation, and embodied
question-answering benchmarks represent a task's goal. It is intentionally
about **question/task generation and storage**, not about implementing a DimSim
evaluator.

The comparison distinguishes three things that are easy to conflate:

1. the **oracle task goal** used to construct and judge an episode;
2. the **goal-conditioned observation or language** exposed to the agent; and
3. an **expert demonstration**, which is one trajectory that reaches the goal,
   not generally the definition of the goal itself.

Only primary sources are used: official repositories and documentation,
released dataset records, and papers from the benchmark authors. Versioned
details are noted where relevant.

## Executive conclusion

The benchmarks do not converge on a single universal `goal` field. They do
converge on a more useful pattern:

```text
task semantics / oracle target
          │
          ├── episode initial condition
          ├── public goal presentation (vector, image, category, or language)
          ├── optional expert trajectory
          └── executable success rule or reference answer
```

For DimSim, this argues against storing only generated English questions. The
generator should first emit a typed, machine-checkable intent and then render
one or more public utterances from it. The typed intent should support at least
three distinct goal kinds:

- **state/destination goals**: make a physical predicate true, such as being
  near or to the left of a sofa;
- **trajectory-conditioned destination goals**: follow a route described in
  language to a hidden endpoint;
- **information goals**: explore as needed and return the answer to a
  scene-grounded question.

LIBERO is the clearest precedent for separating a language instruction from
executable symbolic predicates. Habitat is the clearest precedent for making
the public goal modality configurable while keeping a richer episode goal
private. R2R is the clearest precedent for storing several human utterances
against one route endpoint. EQA and OpenEQA show that an embodied goal can be an
answer rather than a final robot pose.

## Cross-benchmark comparison

| Benchmark | Canonical oracle goal | What the agent receives | Episode/task storage | Success target |
|---|---|---|---|---|
| LIBERO | BDDL task with initial-state and goal predicates; fixed initial simulator states are supplied by the benchmark suite | Task language plus normal robot observations | BDDL file, benchmark task metadata, fixed initial states; demonstrations are separate | Sparse completion when the goal predicates hold |
| robomimic | Defined by the underlying environment/task, not by robomimic's demonstration container | Whatever observations or optional goal observations the configured environment/policy uses | HDF5 trajectories plus environment metadata | Environment reward/success logic; not inferred from the final demonstration frame |
| RoboTwin | Task-specific simulator setup and task-specific success checker | Task language condition and configured observations | Task/config files and trajectory data; exact layout is version/task dependent | Task-specific executable check, aggregated as episode success |
| Habitat PointNav | Goal 3-D position(s), with optional radius and shortest path | Relative point-goal vector, commonly polar/cartesian; no natural language is required | `NavigationEpisode`: scene, start pose, list of goals, optional shortest paths | Stop within configured distance of the goal |
| Habitat ObjectNav | Object category plus one or more object goals/viewpoints in the scene | A category ID/goal sensor, not necessarily a sentence | `ObjectGoalNavEpisode` plus per-scene/category goals | Challenge definition: stop close enough to a valid category instance and at an oracle-visible viewpoint |
| Habitat ImageNav / InstanceImageNav | Goal pose in classic ImageNav; target object instance and goal-image camera parameters in InstanceImageNav | Rendered goal RGB image | Navigation episode plus goal-image metadata | Stop near the goal; InstanceImageNav admits one correct object instance |
| R2R | Last viewpoint of a stored path; path also defines a human route | One of three human natural-language route instructions | JSON: scan, start heading, path, distance, three instructions | Final graph distance below 3 m; SPL also uses shortest/path length |
| REVERIE | Target object ID plus route/goal viewpoint | High-level remote referring expression/instruction | JSON: scan, path, object ID, instructions; separate visible bounding boxes | Navigate near the target and ground the correct object |
| EQA v1 | Reference answer generated from scene semantics; target object/room boxes support grounding and navigation | Natural-language question and egocentric observations | JSON split by house, with question, answer, type, and target/support boxes; navigation paths are separate | Answer accuracy; navigation quality is measured separately |
| OpenEQA | Human reference answer(s) for a question grounded in an episode history | Open-vocabulary natural-language question plus episodic memory, or active access to the environment | JSON question-answer-category records linked to episode histories; active episodes reuse a recorded start state | Natural-language answer match (the release provides an LLM-based protocol) |
| IQUAD / Interactive QA | Answer to a question about a uniquely configured interactive scene | Natural-language question and interactive egocentric environment | Each question is paired with a scene configuration | Correct answer after any required navigation and interaction |

The rows above summarize different benchmark versions. Details and direct
sources follow.

## Manipulation benchmarks

### LIBERO: symbolic goal predicates plus language

LIBERO's official repository exposes a benchmark task with at least three
separate pieces: `task.language`, a BDDL problem file, and a fixed set of
initial simulator states. The environment is constructed from the BDDL file,
while the language is retrieved independently and initial states are selected
with `get_task_init_states`. The repository also states that current evaluation
uses a sparse reward of `+1` when the task is finished
([official LIBERO repository](https://github.com/Lifelong-Robot-Learning/LIBERO)).

The practical representation is therefore not “the demonstration's final
frame.” It is:

```text
BDDL problem
  :objects
  :fixtures / regions
  :init
  :goal = conjunction of symbolic predicates

+ task.language
+ benchmark-supplied initial state
+ separate teleoperation demonstrations
```

LIBERO deliberately groups tasks into Spatial, Object, Goal, and long-horizon
task suites. This separation is itself useful for DimSim: question families can
hold one factor fixed while changing another instead of mixing object identity,
spatial relation, language, and route difficulty into every sample.
Its official collection script records state/action trajectories while also
embedding the BDDL filename/content, problem information, environment metadata,
and simulator model in HDF5 attributes
([LIBERO demonstration collection source](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/master/scripts/libero_100_collect_demonstrations.py)).

Reusable lesson: store an executable semantic goal separately from its
language rendering, and make start-state variation a separate episode layer.

### robomimic: a trajectory container, not a goal schema

robomimic's official HDF5 format stores environment metadata and per-demo
sequences such as states, actions, rewards, dones, observations, and next
observations. The top-level `env_args` names the environment/task and arguments
needed to reconstruct it
([robomimic dataset overview](https://robomimic.github.io/docs/datasets/overview.html)).
The official tooling can replay either stored actions or states in that
environment
([dataset contents and playback](https://robomimic.github.io/docs/tutorials/dataset_contents.html)).

robomimic also supports “goal observations” as a learning input, but that does
not make the last observation in each demonstration the authoritative task
definition. Task completion and sparse/shaped rewards come from the reconstructed
environment
([robomimic environment utilities](https://robomimic.github.io/docs/api/robomimic.utils.html)).
Its environment interface makes the separation explicit with goal accessors
(`get_goal`/`set_goal`) and an executable `is_success` result whose dictionary
contains at least a task-success value
([official robomimic environment API](https://robomimic.github.io/docs/v0.4/api/robomimic.envs.html)).

Reusable lesson: a demonstration record is useful training data and may contain
rewards/dones, but the benchmark corpus still needs an environment-owned goal
or predicate. DimSim should not use “match an expert endpoint” as a generic
goal representation.

### RoboTwin: task-owned generation, separately stored language, and success

RoboTwin 2.0 exposes named tasks, separate YAML task configurations, and
task-specific data collection. The official repository invokes collection with
`task_name + task_config` and describes evaluation across single-task,
visual-robustness, language-diversity, multi-task, and cross-embodiment settings
([official RoboTwin 2.0 repository](https://github.com/robotwin-Platform/robotwin)).
One episode's observations/actions are stored as an HDF5 file, while the
corresponding language is stored separately in an `instructions` directory;
the collection also emits scene information and seeds
([official collection documentation](https://robotwin-platform.github.io/doc/usage/collect-data.html)).
At deployment, the policy loop receives `instruction` as a string from the task
environment
([official policy-deployment interface](https://robotwin-platform.github.io/doc/usage/deploy-your-policy.html)).

RoboTwin explicitly generates both task-level descriptions and per-episode
instructions. Its task-description tool splits generated formulations into
seen and unseen instructions
([official description-generation documentation](https://robotwin-platform.github.io/doc/usage/description.html)).
The paper shows many surface forms attached to one manipulation intent, for
example several ways to state placing one bottle to the left of another
([official RoboTwin 2.0 paper](https://robotwin-platform.github.io/paper.pdf)).

Its key architectural lesson is that the common benchmark layer delegates
scene randomization and completion to each task implementation. It is not
evidence for one universal geometric goal format. Exact data fields and checker
logic should be verified against the pinned RoboTwin version and selected task;
the public repository has changed substantially between RoboTwin releases.

Reusable lesson: a tagged family with family-specific oracle fields is safer
than forcing navigation, QA, and manipulation into one flat `goal_position`
record.

## Navigation benchmarks

### Habitat: one episode structure, several public goal modalities

Habitat's base `NavigationEpisode` stores an episode ID, scene, initial position
and rotation, a list of `NavigationGoal`s, and optional shortest paths. A
`NavigationGoal` has a 3-D position and optional radius
([Habitat navigation task source](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/tasks/nav/nav.py#L60-L96)).
PointNav exposes the goal as a relative cartesian or polar vector rather than as
language
([PointGoal sensor source](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/tasks/nav/nav.py#L99-L191)).

Classic ImageNav uses the same underlying goal position but renders an RGB
observation at that position as the goal sensor
([ImageGoal sensor source](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/tasks/nav/nav.py#L193-L261)).
InstanceImageNav is richer: its episode adds `goal_object_id`,
`goal_image_id`, and `object_category`, while its stored goal carries camera
position, rotation, field of view, resolution, and coverage metadata used to
render the image
([InstanceImageNav source](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/tasks/nav/instance_image_nav_task.py#L35-L87)).

ObjectNav adds `object_category` to the episode. Its dataset loader separately
stores/deduplicates goals by scene/category, including valid object
viewpoints
([ObjectNav dataset source](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/datasets/object_nav/object_nav_dataset.py#L25-L65)).
The goal sensor can expose a category or object ID
([Habitat configuration keys](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/config/CONFIG_KEYS.md)).

Habitat's generic success measure requires an explicit Stop action and distance
below a configured threshold
([success source](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/tasks/nav/nav.py#L461-L499)).
For the 2023 ObjectNav challenge, the official definition was stricter: Stop
within 1 m Euclidean distance of any target-category instance, at a position
from which an oracle can view that object by rotating or looking up/down
([official Habitat Challenge repository](https://github.com/facebookresearch/habitat-challenge)).
These are versioned benchmark policies, not interchangeable constants.

Reusable lessons:

- The stored oracle target may be richer than the public goal presentation.
- A task can share episode plumbing while selecting vector, category, image, or
  language as its public goal modality.
- An object goal is naturally a set of valid viewpoints/regions, not necessarily
  one point at the object's center.

### R2R: a route endpoint with multiple human instructions

R2R stores one JSON item with:

```json
{
  "scan": "...",
  "path_id": 123,
  "path": ["start_viewpoint", "...", "goal_viewpoint"],
  "heading": 1.2,
  "distance": 8.4,
  "instructions": ["...", "...", "..."]
}
```

The first path viewpoint is the start and the last is the goal; three different
human instructions describe the same route
([official R2R task README](https://github.com/peteanderson80/Matterport3DSimulator/blob/master/tasks/R2R/README.md)).
The original evaluator scores final graph distance to that last viewpoint,
using a 3 m success margin, and computes SPL from shortest and executed path
lengths
([official R2R evaluator](https://github.com/peteanderson80/Matterport3DSimulator/blob/master/tasks/R2R/eval.py#L16-L99)).

This is not the same as ObjectNav. R2R language often describes route
procedures and landmarks, while the oracle goal is still a hidden endpoint.
The stored reference path serves both as data-collection context and a shortest
route, but alternative trajectories may succeed.

Reusable lesson: attach several utterances to one semantic/physical task, and
do not duplicate the oracle goal for every paraphrase.

### REVERIE: navigate, then identify a remotely described object

REVERIE combines high-level language, exploration, navigation, and object
grounding. Its released task JSON includes a building, start heading, path,
target `objId`, and multiple instructions; separate per-viewpoint files contain
object visibility and bounding boxes
([official REVERIE repository](https://github.com/YuankaiQi/REVERIE#5-data-organization-of-the-reverie-task)).
The authors describe the goal as navigating in an unseen environment and
identifying a target object from a remote referring expression
([official CVPR paper](https://openaccess.thecvf.com/content_CVPR_2020/html/Qi_REVERIE_Remote_Embodied_Visual_Referring_Expression_in_Real_Indoor_Environments_CVPR_2020_paper.html)).

Reusable lesson: “find the thing described by its role and context” is a
distinct family from category-only ObjectNav and should retain the target
instance plus grounding evidence in the oracle record.

## Exploration and question answering

### EQA v1: programmatically generated questions backed by scene semantics

EmbodiedQA gives the agent a natural-language question and requires it to
navigate from a random start, gather egocentric visual evidence, and answer
([official task repository](https://github.com/facebookresearch/EmbodiedQA)).
Questions are generated programmatically from the SUNCG/House3D scene in a
CLEVR-like manner. The official code lists:

- EQA v1: location, color, and place-preposition questions;
- EQA v1-extended: existence, logical, object-count, room-count, and
  distance-comparison questions.

The released EQA v1 record stores `question`, `answer`, `type`, and bounding
boxes for the target object and supporting room. Splits are disjoint by
environment
([official EQA data page](https://embodiedqa.org/data)).
Shortest navigation paths are distributed separately from the question file
and are used to train or evaluate navigation components
([official EQA repository](https://github.com/facebookresearch/EmbodiedQA#supervised-learning)).

This is an important distinction for DimSim: the **answer and semantic support**
define the information goal; an oracle path is optional supervision, not the
goal itself. The repository evaluates answer top-1 accuracy and also reports
navigation metrics at different initial distances.

Reusable lesson: generate an executable question program against private scene
semantics, store its reference answer and referenced entities, then render the
question. This is much safer than asking an LLM to invent both a question and
its answer.

### OpenEQA: human, open-vocabulary questions linked to episode history

OpenEQA broadens EQA beyond templated attributes. It defines two settings:
answer from an episodic memory, or actively explore from a recorded start state.
The released records contain `question`, `answer`, `category`, `question_id`,
and an `episode_history` identifier; some localization questions also include
multiple acceptable answers
([released OpenEQA JSON](https://github.com/facebookresearch/open-eqa/blob/main/data/open-eqa-v0.json)).

The authors define seven question categories:

1. object recognition;
2. attribute recognition;
3. object-state recognition;
4. object localization;
5. spatial reasoning;
6. functional reasoning;
7. outside-world knowledge.

Questions were human-authored and independently checked for ambiguity,
answerability, and answer correctness
([official OpenEQA paper](https://open-eqa.github.io/assets/pdfs/paper.pdf)).
The official repository describes more than 1,600 question-answer pairs with
corresponding episode histories and provides an LLM-based natural-language
answer evaluator
([official OpenEQA repository](https://github.com/facebookresearch/open-eqa)).

OpenEQA is valuable for question breadth, but its open-vocabulary reference
answers are not automatically executable from simulator semantics. For an
oracle-generated DimSim corpus, object recognition, attributes, states,
localization, and geometric/spatial questions are the easiest to verify.
Functional and world-knowledge questions require external knowledge and
subjective language, so they should be a separate curated track rather than
silently mixed into an exact-oracle track.

### Interactive QA: answering may require changing what is observable

IQUAD/Interactive Question Answering pairs each question with a unique
AI2-THOR scene configuration. Its canonical example, “Are there any apples in
the fridge?”, may require navigating and opening the refrigerator before
answering. The released task contains 75,000 questions and explicitly requires
navigation, visual understanding, interaction, and question-conditioned
planning
([official IQA paper and repository link](https://arxiv.org/abs/1712.03316)).

Reusable lesson: questions about contained or occluded objects form a separate
**interactive information goal**. DimSim should not generate them until the
oracle can encode the required state, visibility, and allowed interactions.

## What goal families this supports for DimSim

The sources support a broader generator than “navigate to the left of the
sofa,” but the families should remain typed rather than becoming an
unstructured bag of prompts.

### A. Destination and grounding goals

| Family | Example public instruction | Private representation |
|---|---|---|
| Point destination | “Go to the marked location.” | Navigable pose/region |
| Category navigation | “Find a sofa and stop near it.” | Category plus valid instance viewpoints |
| Instance navigation | “Go to the red armchair beside the bookshelf.” | Target instance plus referring-expression program |
| Object-relative placement | “Stand on the sofa's left side.” | Reference instance, relation frame, valid goal region |
| Pair-relative placement | “Stand between the sofa and coffee table.” | Two instances plus valid relation region |
| Route following | “Leave the bedroom, turn right at the hall, and stop by the table.” | Start pose, hidden endpoint, optional reference path |
| Remote grounding | “Find the table where I left the blue mug.” | Target instance, remote referring expression, visibility evidence |

Habitat, R2R, and REVERIE show that category, instance, route, and referring
expression goals are meaningfully different benchmark conditions.

### B. Targeted look-and-answer goals

These questions can usually be answered by finding one entity or room:

- object identity: “What is on the coffee table?”
- object attribute: “What color is the armchair?”
- object state: “Is the cabinet door open?”
- object location: “Which room contains the floor lamp?”
- local relation: “What is immediately to the left of the sofa?”
- containment/support: “What is inside/on the shelf?”
- room attribute: “What material is the kitchen floor?”

The private goal should be a query program plus answer, not a target robot pose.
The referenced entities and one or more valid evidence viewpoints can be stored
to validate generation and later diagnose exploration.

### C. Broad-exploration and aggregation goals

These force more exploration than approaching one known target:

- existence: “Is there a television anywhere in the apartment?”
- object count: “How many dining chairs are in the apartment?”
- room count: “How many bedrooms are there?”
- per-room inventory: “Which rooms contain a chair?”
- comparison: “Are there more chairs in the dining room or the office?”
- distance comparison: “Is the sofa closer to the television or the window?”
- topology: “Which rooms are connected directly to the hallway?”
- extrema: “Which room has the most tables?”
- ordered discovery: “Starting here, name the rooms encountered before reaching
  the kitchen.”
- scene summary: “Walk through the apartment and list the large furniture in
  each room.”

EQA v1-extended directly supports existence, logical, count, and distance
comparison families. The last four items are DimSim proposals extrapolated from
the same semantic-query pattern; they are not claimed to be EQA task types.
They require reliable room regions, adjacency, and complete semantic coverage.

A vague prompt such as “walk around the room and answer questions” does not
itself create an exploration benchmark: an agent might guess correctly without
walking. The canonical goal remains the answer. If exploration behavior must
also be required, store that as a separate episode constraint or evidence
requirement rather than embedding it only in prose.

### D. Multi-hop information goals

- “What color is the object on the table nearest the sofa?”
- “Which room contains the chair that is closest to a window?”
- “Is the lamp beside the bed in the same room as the desk?”
- “What object is between the sofa and the television?”

These compile naturally to a query graph or functional program. The generator
should reject a sample if the program returns no answer, several equally valid
answers, or an answer that changes under small geometry tolerances.

### E. Interactive information goals

- “Are any mugs inside the closed cabinet?”
- “Which drawer contains the utensils?”
- “Is the light in the bedroom on?”

These are inspired by Interactive QA and OpenEQA state questions. They should be
deferred unless DimSim's authoritative state and visibility oracle can prove
both the answer and what must be manipulated to observe it.

## Recommended stored representation

The benchmark sources favor a two-layer record: reusable task semantics and an
episode-specific start/variant. A proposed DimSim shape is:

```text
TaskIntent
├── id, scene_revision, generator_version
├── kind
│   ├── destination
│   ├── route_destination
│   ├── inspect_and_answer
│   └── interactive_answer
├── oracle
│   ├── target/reference entity IDs
│   ├── executable relation or query program
│   ├── answer value and acceptable aliases
│   ├── valid goal/evidence regions
│   └── ambiguity and stability margins
└── utterances[]
    ├── text
    ├── provenance/template ID
    └── language

EpisodeVariant
├── task_intent_id + utterance_id
├── start pose / scene state
├── observability and distance band
└── optional reference path or evidence plan
```

For an information goal, the oracle record should be at least:

```json
{
  "kind": "inspect_and_answer",
  "query_program": {
    "op": "count",
    "set": {
      "op": "objects_in_room",
      "object_class": "chair",
      "room_id": "dining_room_1"
    }
  },
  "answer": {
    "type": "integer",
    "value": 4
  },
  "references": ["dining_room_1", "chair_2", "chair_5", "chair_7", "chair_9"],
  "evidence_regions": ["region_dining_room_overview_1"]
}
```

The public record need only contain an opaque task ID and utterance:

```json
{
  "task_id": "task_…",
  "utterance": "Walk through the dining room and tell me how many chairs it has."
}
```

At runtime, the agent should receive the utterance through its normal user
message interface. It should not receive the query program, answer, entity IDs,
semantic inventory, evidence regions, reference path, or template ID.

## Store generated questions or generate them live?

The primary-source patterns favor stored benchmark releases:

- R2R stores several instructions against a fixed path/endpoint.
- LIBERO stores task language and BDDL separately from demonstrations.
- Habitat stores episodes and goals in compressed JSON datasets.
- EQA and OpenEQA store question-answer records linked to scenes or histories.

DimSim should therefore generate candidates offline, validate them, and freeze
accepted records with the scene hash and generator version. Live generation is
useful for development or training augmentation, but it makes comparison,
auditing, ambiguity review, and held-out evaluation much harder.

Natural-language variation should also be stored separately from semantic task
identity. One intent can have:

- one controlled canonical utterance;
- several vetted paraphrases;
- optional human-authored formulations;
- later, adversarial or ambiguous formulations in a clearly labeled track.

This mirrors R2R's multiple instructions per physical route without pretending
that each paraphrase is a different oracle goal.

## Generation recommendations grounded in the comparison

1. **Generate semantics first, language second.** LIBERO and EQA provide the
   strongest precedents: construct executable predicates/query programs from
   the simulator oracle, validate the result, then render language.
2. **Use a tagged union, not one universal goal point.** Habitat's goal
   modalities and the difference between EQA and navigation show why.
3. **Keep demonstration and reference paths optional.** robomimic and R2R show
   they can be useful supervision without defining the only valid solution.
4. **Attach multiple utterances to one intent.** This enables language
   robustness experiments without confounding task geometry.
5. **Separate task from episode difficulty.** Start pose, target visibility,
   route length, room distance, and scene randomization belong in episode
   variants.
6. **Record answer support.** For QA, retain referenced entities and optional
   valid evidence viewpoints so a future evaluator can distinguish a correct
   grounded answer from a lucky guess.
7. **Balance oracle answers and create counterfactuals.** Existence, count, and
   binary state questions are otherwise vulnerable to language-only priors.
8. **Reject uncertain samples.** Referential ambiguity, symmetric object
   orientation, incomplete visibility, boundary-sensitive relations, and
   multiple valid answers should be explicit rejection reasons.
9. **Keep world knowledge separate.** OpenEQA uses it productively, but it
   cannot generally be verified from DimSim asset semantics alone.
10. **Do not force walking through wording alone.** If the benchmark requires
    exploration coverage or evidence collection, represent that independently
    of the answer goal.

## Suggested first question-generation breadth

A first useful corpus need not stop at relative navigation. It could contain:

```text
Destination (35%)
  category / instance / object-relative / pair-relative

Targeted QA (30%)
  identity / attribute / state / room / local relation

Broad exploration QA (25%)
  existence / count / inventory / comparison / room topology

Multi-hop QA (10%)
  two- or three-relation query programs
```

Those percentages are a proposal, not a convention taken from another
benchmark. The release should report counts by task family, answer type,
reference count, room count, required visibility, and query-program depth.

Interactive QA, unrestricted scene summaries, functional reasoning, and
outside-world knowledge should begin as separately curated tracks. They do not
share the same automatic-verification guarantees as semantic and geometric
questions.

## Remaining uncertainties to resolve in DimSim

- Whether asset semantics reliably expose colors, materials, articulated
  states, support/containment, and canonical object orientation.
- Whether rooms and door connectivity exist as authoritative semantics or must
  be derived geometrically.
- Whether the visibility oracle can identify viewpoints that provide sufficient
  evidence, rather than merely line of sight to an object's center.
- Whether duplicate objects can be referred to with stable, perceivable
  descriptions.
- Which query results require tolerance-aware geometry rather than symbolic
  scene metadata.
- Whether generated QA should require a short answer, structured answer, or
  free-form language. EQA uses closed answers; OpenEQA demonstrates broader
  language but correspondingly relies on a semantic evaluator.

These questions can be answered by a semantic-capability inventory over a
pinned DimSim scene before designing the evaluator.
