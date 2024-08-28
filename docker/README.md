# SpecGuard: Docker

We provide two separate Docker containers for the drone and the rover, each designed to installation the necessary dependencies and autopilot software.

## Setup
The Docker containers are designed to mount the `environments`, `drone`, and `rover` directories. This setup allows uers to create and modify configuration files or environment settings outside the containers while seamlessly using them inside the containers. 

1. **NVIDIA Container Toolkit**: Install Docker and NVIDIA Container Toolkit. Installation instructions can be found [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
2. **Environment Binaries**: Ensure that the environment binaries are in the `environments` directory. This allows the binaries to be shared across multiple container instances.
3. **Configuration**: Place all configuration files, including custom AirSim `settings.json` files, in the appropriate directories - `drone` or `rover` directories. 

## Usage

### Building the container

To build the pipelinefor each RAV type, use the following scripts:
```
./build_drone.sh
./build_car.sh
```
To customize the Docker image, edit `DroneDockerfile` or `RoverDockerfile`. Note that the modified autopilot firmware is pulled from the `modified_ardupilot` and `modified_px4` directories within this repository.

### Running the container

To run the drone or rover container, use the corresponding script. You can optionally specify the GPU device for running the container.  
```
./run_drone.sh <gpu_device>
./run_rover.sh <gpu_device>
```
To modify the directories or files that are mounted, update the `run_rover.sh` or `run_drone.sh` scripts.

### Running the Deep-RL policy

To run the Deep-RL policy **set the docker flag**, `-d`, and specify a configuration file.

```
python ppo_drone.py -d -c /path/to/config
python ppo_rover.py -d -c /path/to/config
```

Make sure the configuration file includes the correct paths to your environment binaries.

### Options
  - `-c/--config path/to/config` the path to your configuration file.
  - `-n/--name agent_name` the name of the trained policy.
  - `-lr/--load-recovery` to load a trained policy.
  - `-la/--load-attacker` to load a trained anomaly agent.
  - `-v/--verbose` print verbose output to console for debugging.
  - `-e/--evaluate num_evaluation_episodes` number of evaluation episodes.

### Examples
```
python ppo_drone.py -d -c configs/nh_gps.yaml
python ppo_car.py -d -c configs/nh_gps.yaml
python ppo_drone.py -d -c configs/cb_gps.yaml -lr trained_recovery_model.zip
python ppo_car.py -d -c configs/cb_gps.yaml -lr trained_recovery_model.zip
python ppo_drone.py -d -c configs/nh_gyro.yaml -lr trained_recovery_model.zip -la trained_attacker_model.zip -e 100 -v
python ppo_car.py -d -c configs/nh_gyro.yaml -lr trained_recovery_model.zip -la trained_attacker_model.zip -e 100 -v
```