---
title: "VQA Generation Code Flow"
---

# VQA Generation Code Flow

This flowchart traces one `dimos vqa generate` execution through the Python files, classes, and
functions that implement generation. For command options and artifact schemas, see
[Pipeline](/docs/benchmarking/vqa-generation/pipeline.md). For component responsibilities, see
[Infrastructure](/docs/benchmarking/vqa-generation/infrastructure.md).

```mermaid
flowchart TD
    COMMAND["dimos vqa generate"]

    subgraph CLI["dimos/cli/vqa.py"]
        GENERATE["generate()<br/>Receive Typer options"]
        RESOLVE["_resolve_generation_spec()<br/>Build VqaGenerationSpecification"]
        DISPATCH["execute_generation()<br/>Lazy import and dispatch"]
    end

    subgraph SPEC["generation/specification.py"]
        SPEC_MODEL["VqaGenerationSpecification<br/>Validate frame bounds, mode,<br/>grounding settings, and output"]
    end

    subgraph RUNNER["generation/runner.py"]
        EXECUTE["execute_generation(specification)"]
        PREFLIGHT["Validate API key and CUDA"]
        CONSTRUCT["Construct MoonDream, EdgeTAM,<br/>question author, and adapters"]
        FRAME_LOOP{"For each frame index"}
        RESUME{"Completed frame.json<br/>exists?"}
        VALIDATE["_validate_completed_frame()<br/>Verify generation settings"]
        LOAD["load_go2_frame()"]
        BUILD_PRIMITIVES["FramePerceptionPrimitives(...)<br/>One shared instance per frame"]
        MODE{"question_mode"}
        COLLECT["Collect answered and<br/>rejected results"]
        WRITE_FRAME["write_frame_record(...)"]
        WRITE_MANIFEST["write_dataset_manifest(...)"]
        WRITE_RUN["_write_generation_run(...)"]
    end

    subgraph RECORDING["generation/recording.py"]
        LOAD_FRAME["load_go2_frame()<br/>Read image, point cloud, intrinsics,<br/>and point-cloud-to-camera transform"]
        CALIBRATED["CalibratedFrame"]
    end

    subgraph AUTHORING["generation/question_agent.py"]
        CONSTRAINED_AUTHOR["OpenAIQuestionAgent.propose(image)<br/>Return QuestionIntent list"]
        AGENTIC_AUTHOR["OpenAIFreeformQuestionAuthor.propose(image)<br/>Return QuestionProposal list"]
    end

    subgraph PRIMITIVES["generation/primitives/"]
        FRAME_PRIMITIVES["frame.py<br/>FramePerceptionPrimitives"]
        DETECT["detect_objects()"]
        SEGMENT["segment_detections()<br/>or segment_detection()"]
        GROUND["ground_masks()<br/>or ground_mask()"]
        PROJECTION["projection.py<br/>project_visible_points()"]
        GROUNDING["grounding.py<br/>Mask support to GroundedObject"]
        GEOMETRY["geometry.py<br/>plane, opening, corridor,<br/>and mask-point math"]
        MEASURE["Frame primitive methods<br/>fit planes, measure heights,<br/>relations, ranges, and corridor"]
        CACHE["Frame-scoped caches<br/>detections, masks, canonical objects,<br/>and accepted ground plane"]
    end

    subgraph DETERMINISTIC["Constrained path"]
        ANSWERER_FILE["generation/deterministic_question_answerer.py"]
        ANSWERER["DeterministicQuestionAnswerer.answer(intent)"]
        REGISTRY["FAMILIES registry<br/>Map QuestionKind to family function"]
        FAMILY["generation/families.py<br/>family(intent, context)"]
        CONTEXT["generation/family_context.py<br/>FamilyContext"]
        GROUND_SEQUENCE["FamilyContext.ground(query)<br/>Detect -> segment -> ground<br/>or reuse cached grounding"]
        FAMILY_POLICY["Family-owned sequence<br/>measure -> quality gate -> classify<br/>-> choose or bucket answer"]
        COMMON["generation/family_common.py<br/>Render question and construct<br/>shared rejection result"]
        GROUND_TRUTH["GroundTruthResult"]
    end

    subgraph AGENTIC["Agentic path"]
        TOOLS_FILE["generation/oracle_tools.py"]
        TOOL_REGISTRY["LocalOracleToolRegistry<br/>Expose low-level primitives<br/>through opaque IDs"]
        ORACLE_FILE["generation/oracle.py"]
        ORACLE["PrivateOracle.answer(proposal, tools)<br/>Select iterative tool calls"]
        VALIDATE_ORACLE["Validate answer contract,<br/>citations, and semantic support"]
        ORACLE_RESULT["AcceptedOracleResult<br/>or RejectedOracleResult"]
    end

    subgraph DATASET["generation/dataset.py"]
        SERIALIZE["_evaluation_rows()<br/>Accepted results -> public cases<br/>and private labels"]
        PRIVATE_RESULT["_private_result()<br/>Serialize evidence and traces"]
        ATOMIC["Atomic JSON, JSONL,<br/>and JPEG writes"]
        FRAME_FILES["audit/frame-N/<br/>ground_truth.json<br/>cases.json<br/>labels.json<br/>frame.json written last"]
        ROOT_FILES["assets/frame-N.jpg<br/>cases.jsonl<br/>labels.jsonl<br/>audit/run.json"]
    end

    COMMAND --> GENERATE
    GENERATE --> RESOLVE
    RESOLVE --> SPEC_MODEL
    SPEC_MODEL --> DISPATCH
    DISPATCH --> EXECUTE
    EXECUTE --> PREFLIGHT
    PREFLIGHT --> CONSTRUCT
    CONSTRUCT --> FRAME_LOOP

    FRAME_LOOP --> RESUME
    RESUME -->|Yes| VALIDATE
    VALIDATE -->|Settings match: skip| FRAME_LOOP
    RESUME -->|No or partial| LOAD
    LOAD --> LOAD_FRAME
    LOAD_FRAME --> CALIBRATED
    CALIBRATED --> BUILD_PRIMITIVES
    BUILD_PRIMITIVES --> FRAME_PRIMITIVES
    FRAME_PRIMITIVES --> CACHE
    CALIBRATED --> MODE

    MODE -->|constrained| CONSTRAINED_AUTHOR
    CONSTRAINED_AUTHOR --> ANSWERER_FILE
    ANSWERER_FILE --> ANSWERER
    FRAME_PRIMITIVES --> ANSWERER
    ANSWERER --> REGISTRY
    REGISTRY --> FAMILY
    CONTEXT --> FAMILY
    FAMILY --> GROUND_SEQUENCE
    GROUND_SEQUENCE --> DETECT
    DETECT --> SEGMENT
    SEGMENT --> GROUND
    PROJECTION --> GROUND
    GROUNDING --> GROUND
    GROUND --> CACHE
    GROUND_SEQUENCE --> FAMILY_POLICY
    FAMILY_POLICY --> MEASURE
    GEOMETRY --> MEASURE
    MEASURE --> FAMILY_POLICY
    COMMON --> FAMILY_POLICY
    FAMILY_POLICY --> GROUND_TRUTH
    GROUND_TRUTH --> COLLECT

    MODE -->|agentic| AGENTIC_AUTHOR
    AGENTIC_AUTHOR --> TOOLS_FILE
    TOOLS_FILE --> TOOL_REGISTRY
    FRAME_PRIMITIVES --> TOOL_REGISTRY
    TOOL_REGISTRY --> ORACLE_FILE
    ORACLE_FILE --> ORACLE
    ORACLE -->|Tool call| TOOL_REGISTRY
    TOOL_REGISTRY -->|Typed result or rejection| ORACLE
    ORACLE --> VALIDATE_ORACLE
    VALIDATE_ORACLE --> ORACLE_RESULT
    ORACLE_RESULT --> COLLECT

    COLLECT --> WRITE_FRAME
    WRITE_FRAME --> SERIALIZE
    WRITE_FRAME --> PRIVATE_RESULT
    SERIALIZE --> ATOMIC
    PRIVATE_RESULT --> ATOMIC
    ATOMIC --> FRAME_FILES
    FRAME_FILES --> FRAME_LOOP

    FRAME_LOOP -->|All selected frames handled| WRITE_MANIFEST
    WRITE_MANIFEST --> ROOT_FILES
    WRITE_MANIFEST --> WRITE_RUN
    WRITE_RUN --> ROOT_FILES
```

The primary code boundary is:

- `runner.py` owns execution and model lifecycle.
- `FramePerceptionPrimitives` owns reusable frame-scoped perception, measurements, and caches.
- Constrained functions in `families.py` own complete deterministic answer recipes.
- The agentic oracle selects low-level primitive calls through `LocalOracleToolRegistry`.
- `dataset.py` owns serialization and atomic publication; it does not generate answers.
