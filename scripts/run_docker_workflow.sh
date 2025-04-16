#!/bin/bash

# Script to run GitHub Actions workflow locally using act
# Usage: ./run_docker_workflow.sh [push]
# If 'push' argument is provided, it will simulate a workflow with push=true

cd "$(dirname "$0")/.."

if [ "$1" == "push" ]; then
  echo "Running workflow with push=true"
  act -W .github/workflows/docker-build.yml --secret-file .act.secrets -e <(echo '{"inputs":{"push":"true"}}') workflow_dispatch
else
  echo "Running workflow with push=false (default)"
  act -W .github/workflows/docker-build.yml --secret-file .act.secrets workflow_dispatch
fi

# Note: Make sure to create .act.secrets file with:
# DOCKERHUB_USERNAME=your_username
# DOCKERHUB_TOKEN=your_token