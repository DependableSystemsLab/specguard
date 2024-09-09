#!/bin/bash

# Define the image name
IMAGE_NAME=drl-drone-docker

# Define the Dockerfile name
DOCKERFILE=DroneDockerfile

# Rebuild the image if required
docker build --no-cache -t $IMAGE_NAME -f $DOCKERFILE .