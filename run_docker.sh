#!/bin/bash

# Check if an argument was provided
if [ $# -gt 0 ]; then
  option=$1
else
  echo "Select an option:"
  echo "1) Docker compose sequence for ros_agents: Takes down containers, builds, then brings them up."
  echo "2) Attach to tmux session: Exec into the container and attach to the 'python_session'."
  echo "3) Remove existing container/image for ros_agents, then Docker compose sequence without cache."
  echo "4) Docker compose sequence for interface: Takes down containers, builds, then brings them up."
  echo "5) Remove existing container/image for interface, then Docker compose sequence without cache."
  read -p "Enter option (1, 2, 3, 4, or 5): " option
fi

case $option in
  1)
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml down && \
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml build && \
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml up
    ;;
  2)
    docker exec -it ros_agents-dimos-unitree-ros-agents-1 tmux attach-session -t python_session
    ;;
  3)
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml down --rmi all && \
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml build --no-cache && \
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml up
    ;;
  4)
    docker compose -f ./docker/interface/docker-compose.yml down && \
    docker compose -f ./docker/interface/docker-compose.yml build && \
    docker compose -f ./docker/interface/docker-compose.yml up
    ;;
  5)
    docker compose -f ./docker/interface/docker-compose.yml down --rmi all && \
    docker compose -f ./docker/interface/docker-compose.yml build --no-cache && \
    docker compose -f ./docker/interface/docker-compose.yml up
    ;;
  *)
    echo "Invalid option. Please run the script again and select 1, 2, 3, 4, or 5."
    ;;
esac
