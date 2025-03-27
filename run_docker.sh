
#!/bin/bash

# Check if an argument was provided
if [ $# -gt 0 ]; then
  option=$1
else
  echo "Select an option:"
  echo "1) Docker compose sequence: Takes down containers, builds, then brings them up."
  echo "2) Attach to tmux session: Exec into the container and attach to the 'python_session'."
  echo "3) Docker compose sequence: Takes down containers, builds, then brings them up."
  echo "4) Build and run web-os container"
  echo "5) Build and run agent container"
  echo "6) Build and run semantic-seg model container"
  echo "7) Build and run semantic-seg robot agent container"
  read -p "Enter option (1-7): " option
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
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml down --rmi all -v && \
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml build --no-cache && \
    docker compose -f ./docker/unitree/ros_agents/docker-compose.yml up
    ;;
  4)
    docker compose -f ./docker/interface/docker-compose.yml down && \
    docker compose -f ./docker/interface/docker-compose.yml build && \
    docker compose -f ./docker/interface/docker-compose.yml up
    ;;
  5)
    docker compose -f ./docker/agent/docker-compose.yml down && \
    docker compose -f ./docker/agent/docker-compose.yml build && \
    docker compose -f ./docker/agent/docker-compose.yml up
    ;;
  6)
    docker compose -f ./docker/models/semantic_seg/docker-compose.yml down && \
    docker compose -f ./docker/models/semantic_seg/docker-compose.yml build && \
    docker compose -f ./docker/models/semantic_seg/docker-compose.yml up
    ;;
  7)
    docker compose -f ./docker/unitree/ros_dimos_seg/docker-compose.yml down && \
    docker compose -f ./docker/unitree/ros_dimos_seg/docker-compose.yml build && \
    docker compose -f ./docker/unitree/ros_dimos_seg/docker-compose.yml up
    ;;
  8)
    docker compose -f ./docker/models/hugging_face_local/docker-compose.yml down && \
    docker compose -f ./docker/models/hugging_face_local/docker-compose.yml build && \
    docker compose -f ./docker/models/hugging_face_local/docker-compose.yml up
    ;;
  *)
    echo "Invalid option. Please run the script again and enter a number between 1 and 5."
    ;;
esac