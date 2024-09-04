#!/bin/bash

# Define the image name
IMAGE_NAME=drl-rover-docker

# Define the Dockerfile name
DOCKERFILE=RoverDockerfile

# Rebuild the image if required
docker build -t $IMAGE_NAME -f $DOCKERFILE .