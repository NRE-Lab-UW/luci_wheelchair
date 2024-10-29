# ROS2 Container ID
ROS2_CONTAINER_ID=""

# Function to run the Docker container interactively with a mounted volume
function ws-start {
    local workspace_dir=$(pwd)
    ROS2_CONTAINER_ID=$(docker run -d -it -v $SSH_AUTH_SOCK:/ssh-agent -e SSH_AUTH_SOCK=/ssh-agent -v $workspace_dir/src:/root/nre_luci_ws/src -p 8765:8765 luci.jfrog.io/ros2-sdk-docker-local/luci-ros2-sdk:latest)
    docker exec "$ROS2_CONTAINER_ID" apt install python3-pip -y
    docker exec "$ROS2_CONTAINER_ID" pip install setuptools==58.2.0
    echo "Updated setuptools package to allow colcon build to run without warnings"
    echo "Started container with ID: $ROS2_CONTAINER_ID"

}

# Function to attach to the running container using
function ws-exec {    
    if [ -n "$ROS2_CONTAINER_ID" ]; then
        docker exec -it -w /root/nre_luci_ws "$ROS2_CONTAINER_ID" bash 
    else
        echo "No container is currently running. Start a container first."
    fi    
}

# Function to stop and remove the running container
function ws-stop {
    if [ -n "$ROS2_CONTAINER_ID" ]; then
        docker stop "$ROS2_CONTAINER_ID" && docker rm "$ROS2_CONTAINER_ID"
        echo "Stopped and removed container with ID: $ROS2_CONTAINER_ID"
        ROS2_CONTAINER_ID=""
    else
        echo "No container is currently running."
    fi
}
