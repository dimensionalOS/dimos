#!/bin/bash

# Function to display menu options
show_menu() {
  echo "🐳 DIMOS Runner 🐳"
  echo "=================================="
  echo "Available commands:"
  echo "  0 | unitree     : Build and run dimOS agent & interface with unitree go2"
  echo "  1 | web         : Build and run web-os container"
  echo "  2 | hf-local    : Build and run huggingface local model"
  echo "  3 | hf-remote   : Build and run huggingface remote model"
  echo "  4 | gguf        : Build and run ctransformers-gguf model"
  echo "=================================="
}

# Function to run docker compose commands
run_docker_compose() {
  local file=$1
  local rebuild=$2

  if [ "$rebuild" = "full" ]; then
    echo "📦 Full rebuild with --no-cache..."
    docker compose -f $file down --rmi all -v && \
    docker compose -f $file build --no-cache && \
    docker compose -f $file up
  else
    echo "🚀 Building and running containers..."
    docker compose -f $file down && \
    docker compose -f $file build && \
    docker compose -f $file up
  fi
}

# Check if an argument was provided
if [ $# -gt 0 ]; then
  option=$1
else
  show_menu
  read -p "Enter option (number or command): " option
fi

# Process the option - support both numbers and text commands
case $option in
  0|unitree)
    run_docker_compose "./docker/unitree/agents_interface/docker-compose.yml"
    ;;
  1|web)
    run_docker_compose "./docker/interface/docker-compose.yml"
    ;;
  2|hf-local)
    run_docker_compose "./docker/models/huggingface_local/docker-compose.yml"
    ;;
  3|hf-remote)
    run_docker_compose "./docker/models/huggingface_remote/docker-compose.yml"
    ;;
  4|gguf)
    run_docker_compose "./docker/models/ctransformers_gguf/docker-compose.yml"
    ;;
  help|--help|-h)
    show_menu
    ;;
  *)
    echo "❌ Invalid option: $option"
    show_menu
    exit 1
    ;;
esac