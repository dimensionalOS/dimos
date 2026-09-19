#!/bin/bash
# The nav benchmark as independent containers: one per (arm, scene), N at a time.
# Each container runs that scene's cases in order with the stock EvalRunner; results land
# under $EVAL_RUNS_DIR (docker/evals/compose.yaml mounts it on /state). Re-run to resume:
# an (arm, scene) with a marker in $EVAL_RUNS_DIR/nav-done is skipped.
#   docker/evals/nav_matrix.sh [-j 12] [-a arm,arm] [-s scene,scene]
# Keys (TYPESAFE_API_KEY, OPENAI_API_KEY, ANTHROPIC_API_KEY) come from the environment.
set -u
cd "$(dirname "$0")/../.."
J=8; ARMS_SEL=""; SCENES_SEL=""
while getopts "j:a:s:" o; do case $o in j) J=$OPTARG;; a) ARMS_SEL=$OPTARG;; s) SCENES_SEL=$OPTARG;; esac; done
export EVALS_IMAGE=${EVALS_IMAGE:-dimensional/evals:nav}
export COMPOSE_FILE=${COMPOSE_FILE:-docker/evals/compose.yaml:docker/evals/compose.gpu.yaml:docker/evals/compose.habitat-data.yaml}
export HABITAT_DATA_DIR=${HABITAT_DATA_DIR:-/data/habitat} EVAL_RUNS_DIR=${EVAL_RUNS_DIR:-$HOME/eval-runs}
export DIMOS_TRANSPORT=zenoh RERUN_SAVE=0
# The HSSD ground truth (PR 4214) is not in the image; mount it from this checkout. NAV_VOLUMES adds more.
export NAV_VOLUMES="-v $PWD/misc/habitat/ground_truth:/app/misc/habitat/ground_truth:ro ${NAV_VOLUMES:-}"
declare -A ARGS=(
  [dimos-planner]="--agent dimos.evals.agents.topic --set send=goal --set send_type=point --set done=goal_reached --set done_type=Bool --set done_when_still=true"
  [typesafe]="--agent dimos.evals.agents.topic --set modules=[\"type-safe-agent\"] --set trace=TypeSafeAgent"
  [pi-astra]="--agent dimos.evals.agents.pi --set no_dimos=true --set provider=openai --set model=gpt-6-astra --set max_steps=100"
  [pi-gpt56]="--agent dimos.evals.agents.pi --set no_dimos=true --set provider=openai --set model=gpt-5.6-luna --set max_steps=100"
  [pi-fable]="--agent dimos.evals.agents.pi --set no_dimos=true --set provider=anthropic --set model=claude-fable-5-1 --set max_steps=100"
  [dimcode-astra]="--agent dimos.evals.agents.dimcode --set provider=openai --set model=gpt-6-astra --set max_steps=100"
  [dimcode-fable]="--agent dimos.evals.agents.dimcode --set provider=anthropic --set model=claude-fable-5-1 --set max_steps=100"
  [pi-opus]="--agent dimos.evals.agents.pi --set no_dimos=true --set provider=anthropic --set model=claude-opus-4-7 --set max_steps=100"
)
declare -A TIMEOUT=([dimos-planner]=300 [typesafe]=300)
arms=${ARMS_SEL:-dimos-planner,typesafe,pi-astra,pi-gpt56,pi-fable,dimcode-astra,dimcode-fable,pi-opus}
scenes=${SCENES_SEL:-$(ls dimos/evals/suites/scenes/habitat/*.json | xargs -n1 basename | sed 's/\.json$//' | paste -sd,)}
mkdir -p "$EVAL_RUNS_DIR/nav-done" "$EVAL_RUNS_DIR/nav-logs" "$EVAL_RUNS_DIR/nav-args"
for arm in "${!ARGS[@]}"; do echo "${ARGS[$arm]}" > "$EVAL_RUNS_DIR/nav-args/$arm"; done
job() {  # one (arm, scene) container, foreground; marker on a clean exit
  arm=$1; scene=$2; timeout=$3; log=$EVAL_RUNS_DIR/nav-logs/$arm--$scene.log
  [ -f "$EVAL_RUNS_DIR/nav-done/$arm--$scene" ] && return 0
  echo "$(date +%FT%T) START $arm $scene"
  docker compose run --rm --name "nav-$arm-$scene-$$" -e DIMOS_ZENOH_SHM=0 -e CI=1 -e PYTEST_VERSION=1 \
    -e "DIMOS_EVAL_TIMEOUT_S=$timeout" $NAV_VOLUMES worker \
    dimos evals run dimos.evals.suites.habitat_nav --tags "$scene" --video $(cat "$EVAL_RUNS_DIR/nav-args/$arm") > "$log" 2>&1
  rc=$?; run=$(grep -a -o "/state/[^ ]*run-[^ ]*" "$log" | tail -1)
  echo "$(date +%FT%T) DONE $arm $scene rc=$rc $run"
  [ "$rc" = 0 ] && [ -n "$run" ] && echo "$run" > "$EVAL_RUNS_DIR/nav-done/$arm--$scene"
}
export -f job NAV_VOLUMES
export NAV_VOLUMES
for arm in ${arms//,/ }; do for scene in ${scenes//,/ }; do echo "$arm $scene ${TIMEOUT[$arm]:-900}"; done; done \
  | xargs -P "$J" -L 1 bash -c 'job "$0" "$1" "$2"' 2>&1 | tee -a "$EVAL_RUNS_DIR/nav-matrix.log"
