#!/bin/bash
# The nav benchmark as independent containers: one per (arm, scene), N at a time.
# Each container runs that scene's cases in order with the stock EvalRunner; results land
# under $EVAL_RUNS_DIR (docker/evals/compose.yaml mounts it on /state). Re-run to resume:
# an (arm, scene) with a marker in $EVAL_RUNS_DIR/nav-done is skipped.
#   docker/evals/nav_matrix.sh [-j 12] [-a arm,arm] [-s scene,scene]
# Keys (TYPESAFE_API_KEY, OPENAI_API_KEY, ANTHROPIC_API_KEY; GLINER_BASE_URL and GLINER_API_KEY for the
# gliner arm, the same agent on the GLiNER endpoint) come from the environment.
set -u
cd "$(dirname "$0")/../.."
J=8; ARMS_SEL=""; SCENES_SEL=""
while getopts "j:a:s:" o; do case $o in j) J=$OPTARG;; a) ARMS_SEL=$OPTARG;; s) SCENES_SEL=$OPTARG;; esac; done
export EVALS_IMAGE=${EVALS_IMAGE:-dimensional/evals:nav}
export COMPOSE_FILE=${COMPOSE_FILE:-docker/evals/compose.yaml:docker/evals/compose.gpu.yaml:docker/evals/compose.habitat-data.yaml}
export HABITAT_DATA_DIR=${HABITAT_DATA_DIR:-/data/habitat} EVAL_RUNS_DIR=${EVAL_RUNS_DIR:-$HOME/eval-runs}
export DIMOS_TRANSPORT=zenoh RERUN_SAVE=0
# The HSSD ground truth (PR 4214) is not in the image; mount it from NAV_GROUND_TRUTH (default: this
# checkout; use a copy outside git, a reset under a live bind mount empties it for running containers).
export NAV_VOLUMES="-v ${NAV_GROUND_TRUTH:-$PWD/misc/habitat/ground_truth}:/app/misc/habitat/ground_truth:ro ${NAV_VOLUMES:-}"
declare -A ARGS=(
  [dimos-planner]="--agent dimos.evals.agents.topic --set send=goal --set send_type=point --set done=goal_reached --set done_type=Bool --set done_when_still=true"
  [typesafe]="--agent dimos.evals.agents.topic --set modules=[\"type-safe-navigation-agent\"] --set trace=TypeSafeNavigationAgent"
  [gliner]="--agent dimos.evals.agents.topic --set modules=[\"type-safe-navigation-agent\"] --set trace=TypeSafeNavigationAgent"
  [pi-astra]="--agent dimos.evals.agents.pi --set no_dimos=true --set provider=openai --set model=gpt-6-astra --set max_steps=100"
  [pi-gpt56]="--agent dimos.evals.agents.pi --set no_dimos=true --set provider=openai --set model=gpt-5.6-luna --set max_steps=100"
  [pi-fable]="--agent dimos.evals.agents.pi --set no_dimos=true --set provider=anthropic --set model=claude-fable-5-1 --set max_steps=100"
  [dimcode-astra]="--agent dimos.evals.agents.dimcode --set provider=openai --set model=gpt-6-astra --set max_steps=100"
  [dimcode-fable]="--agent dimos.evals.agents.dimcode --set provider=anthropic --set model=claude-fable-5-1 --set max_steps=100"
  [pi-opus]="--agent dimos.evals.agents.pi --set no_dimos=true --set provider=anthropic --set model=claude-opus-4-7 --set max_steps=100"
  [dimcode-opus]="--agent dimos.evals.agents.dimcode --set provider=anthropic --set model=claude-opus-4-7 --set max_steps=100"
  [dimcode-gpt56]="--agent dimos.evals.agents.dimcode --set provider=openai --set model=gpt-5.6-luna --set max_steps=100"
)
# Per-arm image override (e.g. TYPESAFE_IMAGE=dimensional/evals:nav-state); others use EVALS_IMAGE.
declare -A IMAGE=([typesafe]="${TYPESAFE_IMAGE:-}")
declare -A TIMEOUT=([dimos-planner]=${NAV_SHORT_TIMEOUT_S:-300} [typesafe]=${NAV_SHORT_TIMEOUT_S:-300} [gliner]=${NAV_SHORT_TIMEOUT_S:-300})
arms=${ARMS_SEL:-dimos-planner,typesafe,pi-astra,pi-gpt56,pi-fable,dimcode-astra,dimcode-fable,pi-opus,dimcode-opus,dimcode-gpt56}
scenes=${SCENES_SEL:-$(ls dimos/evals/suites/scenes/habitat/*.json | xargs -n1 basename | sed 's/\.json$//' | paste -sd,)}
mkdir -p "$EVAL_RUNS_DIR/nav-done" "$EVAL_RUNS_DIR/nav-logs" "$EVAL_RUNS_DIR/nav-args"
for arm in "${!ARGS[@]}"; do echo "${ARGS[$arm]}" > "$EVAL_RUNS_DIR/nav-args/$arm"; done
job() {  # one (arm, scene) container, foreground; marker on a clean exit
  arm=$1; scene=$2; timeout=$3; image=$4; log=$EVAL_RUNS_DIR/nav-logs/$arm--$scene.log
  extra=""; [ "$arm" = gliner ] && extra="-e TYPESAFE_BASE_URL=$GLINER_BASE_URL -e TYPESAFE_API_KEY=$GLINER_API_KEY"
  [ -f "$EVAL_RUNS_DIR/nav-done/$arm--$scene" ] && return 0
  echo "$(date +%FT%T) START $arm $scene"
  name="nav-$arm-$scene-$$"
  # A scene is at most 8 cases; a container past 2 h is wedged (a runner that never noticed its sim died).
  EVALS_IMAGE=$image timeout --foreground 7200 docker compose run --rm --name "$name" -e DIMOS_ZENOH_SHM=0 -e CI=1 -e PYTEST_VERSION=1 \
    -e "DIMOS_EVAL_TIMEOUT_S=$timeout" $extra $NAV_VOLUMES worker \
    dimos evals run dimos.evals.suites.habitat_nav --tags "$scene" --video $(cat "$EVAL_RUNS_DIR/nav-args/$arm") > "$log" 2>&1
  rc=$?; [ "$rc" = 124 ] && docker rm -f "$name" >/dev/null 2>&1
  run=$(grep -a -o "/state/[^ ]*run-[^ ]*" "$log" | tail -1)
  metrics=$(find "${run/#\/state/$EVAL_RUNS_DIR}" -name nav_metrics.json 2>/dev/null | wc -l)
  echo "$(date +%FT%T) DONE $arm $scene rc=$rc $run metrics=$metrics"
  # A marker only for a run that graded something: the runner exits 0 when every case errors.
  [ "$rc" = 0 ] && [ -n "$run" ] && [ "$metrics" -gt 0 ] && echo "$run" > "$EVAL_RUNS_DIR/nav-done/$arm--$scene"
}
export -f job
export NAV_VOLUMES GLINER_BASE_URL GLINER_API_KEY
for arm in ${arms//,/ }; do for scene in ${scenes//,/ }; do echo "$arm $scene ${TIMEOUT[$arm]:-900} ${IMAGE[$arm]:-$EVALS_IMAGE}"; done; done \
  | xargs -P "$J" -L 1 bash -c 'job "$0" "$1" "$2" "$3"' 2>&1 | tee -a "$EVAL_RUNS_DIR/nav-matrix.log"
