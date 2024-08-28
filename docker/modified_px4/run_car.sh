#!/bin/bash

# Define the image name
IMAGE_NAME=drl-car-docker

# Define the network base name
NETWORK_BASE_NAME=airsim_network

# Generate a unique ID for this session
UNIQUE_ID=$(date +%s)

# Construct the network name with the unique ID
NETWORK_NAME=${NETWORK_BASE_NAME}_$UNIQUE_ID

# Create a docker network
docker network create $NETWORK_NAME

# Run the container with the newly created network
docker run -it --rm --gpus '"device=1"' \
    --network $NETWORK_NAME \
    -v $(pwd)/../environments/:/home/airsim_user/environments \
    -v $(pwd)/../car/:/home/airsim_user/car \
    -v $(pwd)/../car/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
    -v $(pwd)/rover.parm:/home/airsim_user/ardupilot/rover.parm \
    
    $IMAGE_NAME

# Remove the docker network after use
docker network rm $NETWORK_NAME
