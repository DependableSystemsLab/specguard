#!/bin/bash

# Check if a GPU device is provided
GPU_DEVICE=$1

# Define the image name
IMAGE_NAME=drl-drone-docker

# Define the network base name
NETWORK_BASE_NAME=airsim_network

# Generate a unique ID for this session
UNIQUE_ID=$(date +%s)

# Construct the network name with the unique ID
NETWORK_NAME=${NETWORK_BASE_NAME}_$UNIQUE_ID

# Make output directory and change permissions
OUT_DIR="$(pwd)/../drone/out"
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
        -v $(pwd)/../drone/:/home/airsim_user/drone \
        -v $(pwd)/../drone/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
        $IMAGE_NAME
else
    # GPU device specified
    docker run -it --rm --gpus "\"device=$GPU_DEVICE\"" \
        --network $NETWORK_NAME \
        -v $(pwd)/../environments/:/home/airsim_user/environments \
        -v $(pwd)/../drone/:/home/airsim_user/drone \
        -v $(pwd)/../drone/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
        $IMAGE_NAME
fi

# Remove the docker network after use
docker network rm $NETWORK_NAME
