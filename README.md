# Specification Aware Recovery for Robotic Autonomous Vehicles

Code for the paper "SpecGuard: Specification Aware Recovery for Robotic Autonomous Vehicles from Physical Attacks", CCS'2024. 

**Key Features**
- Mission specification compliant *safe* recovery for robotic autonomous vehicles (RAV).
- Integrate Deep-RL with autopilot software such as PX4 and ArduPilot.
- Training pipeline to learn a recovery control policy that maintains mission specification compliance even under physical attacks.
- Multi-agent training to optimize the recovery control policy. 

## Dependencies and Prerequisites
We developed and tested on Ubuntu 20.04 and Python 3.8. The following are the dependencies and prerequisites. 

1. **AirSim**: AirSim is built on Unreal engine, and it provides physically and visually realistic simulations of the environment. Follow the instructions provided by Microsoft to [build and setup the AirSim simulator](https://microsoft.github.io/AirSim/build_linux/). 
4. **Create a Virtual Environment**: 
    - Create a new Python virtual environment: `python3.8 -m venv .env`. 
    - Activate the virtual environment: `source .env/bin/activate`. 
    - Install the required dependencies: `pip install -r requirements.txt`.
5. **Install Environments**: The `environments` directory contains scripts for downloading RAV operating environments (e.g., suburban, urban city block, downtown, etc.). 
6. **Setup: Docker or Local Installation**: Additional setup steps for different RAV types are in `drone` and `rover` directory. We also provide docker containers to run our solutions, details are in `docker` directory.
