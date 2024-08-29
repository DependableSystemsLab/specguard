#!/bin/bash

rm -rf PX4

# Create the PX4 directory and enter it
mkdir -p PX4
cd PX4

# Clone the PX4-Autopilot repository recursively
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Run the setup script from the PX4-Autopilot repository
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# Enter the PX4-Autopilot directory
cd PX4-Autopilot

git checkout v1.13.3

git submodule sync --recursive
git submodule update --init --recursive

cp ../../../drone/modified_px4/EKF2.cpp src/modules/ekf2/EKF2.cpp
cp ../../../drone/modified_px4/PositionControl.cpp src/modules/mc_pos_control/PositionControl/PositionControl.cpp

echo "PX4 setup and cloning complete."