# Stable container name so we can find it across terminals
ROS2_CONTAINER_NAME="luci_ros2_sdk"
ROS2_IMAGE="luci.jfrog.io/ros2-sdk-docker-local/luci-ros2-sdk:latest"
ROS2_CONTAINER_ID=""

# Get the ID of the running container with the expected name
function ws_container_id {
    local by_name
    by_name=$(docker ps -q --filter "name=^/${ROS2_CONTAINER_NAME}$" | head -n1)
    if [[ -n "$by_name" ]]; then
        echo "$by_name"
        return 0
    fi

    # Fallback: pick the first running container for the target image (covers runs without the stable name)
    docker ps -q --filter "ancestor=${ROS2_IMAGE}" | head -n1
}

# Function to run the Docker container interactively with a mounted volume
function ws-up {
    local workspace_dir
    workspace_dir=$(pwd)
    local running_id
    running_id=$(ws_container_id)

    # Build SSH mount args only if SSH_AUTH_SOCK is set
    local ssh_args=()
    if [[ -n "$SSH_AUTH_SOCK" ]]; then
        ssh_args=(-v "$SSH_AUTH_SOCK:/ssh-agent" -e SSH_AUTH_SOCK=/ssh-agent)
    fi

    if [[ -n "$running_id" ]]; then
        local running_name
        running_name=$(docker inspect --format='{{.Name}}' "$running_id" 2>/dev/null | sed 's#^/##')
        echo "Container already running (ID: $running_id, name: ${running_name:-unknown}). Reusing existing container."
        ROS2_CONTAINER_ID="$running_id"
        return 0
    fi

    # Remove stopped container with the same name to avoid name collisions
    local stopped_id
    stopped_id=$(docker ps -aq --filter "name=^/${ROS2_CONTAINER_NAME}$")
    if [[ -n "$stopped_id" ]]; then
        docker rm "$stopped_id" >/dev/null || { echo "Failed to remove old container"; return 1; }
    fi

    ROS2_CONTAINER_ID=$(
        docker run -d -it \
          --name "$ROS2_CONTAINER_NAME" \
          "${ssh_args[@]}" \
          -v "$workspace_dir/src:/root/nre_luci_ws/src" \
          -v "$workspace_dir/launch:/root/nre_luci_ws/launch" \
          -p 8765:8765 \
          "$ROS2_IMAGE"
    ) || { echo "Failed to start container"; return 1; }

    docker exec "$ROS2_CONTAINER_ID" apt update && \
    docker exec "$ROS2_CONTAINER_ID" apt install -y python3-pip && \
    docker exec "$ROS2_CONTAINER_ID" pip install setuptools==58.2.0
    echo "Updated setuptools package to allow colcon build to run without warnings"
    echo "Started container with ID: $ROS2_CONTAINER_ID"
}


# Function to attach to the running container using
function ws-exec {    
    local cid
    cid=$(ws_container_id)
    if [[ -z "$cid" ]]; then
        echo "No container is currently running. Start a container first."
        return 1
    fi

    docker exec -it -w /root/nre_luci_ws "$cid" bash 
}

# Function to stop and remove the running container
function ws-down {
    local cid
    cid=$(ws_container_id)
    if [[ -z "$cid" ]]; then
        echo "No container is currently running."
        return 0
    fi

    docker stop "$cid" && docker rm "$cid"
    echo "Stopped and removed container with ID: $cid"
    ROS2_CONTAINER_ID=""
}
