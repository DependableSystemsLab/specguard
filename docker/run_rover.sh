#!/bin/bash

# Get the GPU to use
GPU_DEVICE=$1

# Define the image name
IMAGE_NAME=drl-rover-docker

# Define the network base name
NETWORK_BASE_NAME=airsim_network

# Generate a unique ID for this session
UNIQUE_ID=$(date +%s)

# Construct the network name with the unique ID
NETWORK_NAME=${NETWORK_BASE_NAME}_$UNIQUE_ID

# Make output directory and change permissions
OUT_DIR="$(pwd)/../rover/out"
mkdir -p "$OUT_DIR"
chmod 777 "$OUT_DIR"

# Create a docker network
docker network create $NETWORK_NAME

# Check if GPU device is specified and conditionally add the --gpus flag
if [ -z "$GPU_DEVICE" ]; then
    # No GPU device specified
    docker run -it --rm \
        --network $NETWORK_NAME \
        -v $(pwd)/../environments/:/home/airsim_user/environments \
        -v $(pwd)/../rover/:/home/airsim_user/rover \
        -v $(pwd)/../rover/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
        -v $(pwd)/../rover/rover.parm:/home/airsim_user/ardupilot/rover.parm \
        $IMAGE_NAME
else
    # GPU device specified
    docker run -it --rm --gpus "\"device=$GPU_DEVICE\"" \
        --network $NETWORK_NAME \
        -v $(pwd)/../environments/:/home/airsim_user/environments \
        -v $(pwd)/../rover/:/home/airsim_user/rover \
        -v $(pwd)/../rover/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
        -v $(pwd)/../rover/rover.parm:/home/airsim_user/ardupilot/rover.parm \
        $IMAGE_NAME
fi

# Remove the docker network after use
docker network rm $NETWORK_NAME