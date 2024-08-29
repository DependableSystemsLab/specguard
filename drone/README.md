# SpecGuard: Drone

We integrate PX4 autopilot with Microsoft AirSim o train Deep-RL recovery control policy for drones. 

## Installation and Setup

1. **PX4 Firmware**: Follow the instructions for installing and building PX4 with Airsim [AirSim Docs for PX4](https://microsoft.github.io/AirSim/px4_sitl/). **Checkout version v1.13.3**, `git checkout v1.13.3`.
2. **Copy AirSim Settings**: Copy the `settings.json` into the AirSim directory: `cp settings.json /home/$USER$/Documents/AirSim/settings.json`.
3. **Modify the PX4 Firmware**: Copy all files in `modified_px4` into their respective directories in the PX4 firmware:
   - Copy `EKF2.cpp` into the PX4 Autopilot EKF2 module directory: `cp modified_px4/EKF2.cpp /PX4/PX4-Autopilot/src/modules/ekf2/EKF2.cpp`.
   - Copy `PositionControl.cpp` into the PX4 Autopilot PositionControl module directory: `cp modified_px4/PositionControl.cpp /PX4/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PositionControl.cpp`.
4. **Setup Configuration Files**: Set the following four fields:
    - `home_directory`: the relative home directory of the project
    - `airsim_directory`: the directory of your AirSim installation
    - `px4_directory`: the directory of your PX4 installation
    - `airsim_binary_name`: the name of the AirSim binary 
```
Paths:
  home_directory: /home/YOUR_USER_HERE
  airsim_directory: /airsim-envs/DowntownCity/DowntownCity1/LinuxNoEditor/
  airsim_binary_name: DowntownCity
  px4_directory: /PX4/PX4-Autopilot
  mission_file: ./missions/dc1-box-1.txt
  agent_name: dc_gyro_drone
``` 

## Usage

To run the pipeline, you need to specify a configuration file,
```
python ppo_drone.py -c /path/to/config
```

Ensure the configuration file is properly configured with the paths to your environment binaries and installations. 

### Options
  - `-c/--config path/to/config` the path to your configuration file.
  - `-n/--name agent_name` the name of the trained policy.
  - `-lr/--load-recovery` to load a trained policy.
  - `-la/--load-attacker` to load a trained anomaly agent.
  - `-v/--verbose` print verbose output to console for debugging.
  - `-e/--evaluate num_evaluation_episodes` number of evaluation episodes.

### Examples
```
python ppo_drone.py -c configs/nh_gps.yaml
python ppo_drone.py -c configs/cb_gps.yaml -lr trained_recovery_model.zip
python ppo_drone.py -c configs/nh_gyro.yaml -lr trained_recovery_model.zip -la trained_attacker_model.zip -e 100 -v
```

## Configuration Files

Configuration files specify the environment, attack settings, and training hyperparameters. 

### Parameters

```
Hyperparameters:
    gamma: Discount factor for future rewards.
    clip_ratio: Used to limit policy updates, ensuring changes aren't too drastic.
    value_function_learning_rate: Learning rate for the value function.
    lam: Lambda parameter for Generalized Advantage Estimation (GAE).
    save_freq: Frequency (in timesteps) at which the policy is saved.
    train_steps: Total number of training steps.

Connections:
    airsim_ip: IP of the AirSim simulator.
    px4_ip: IP of the PX4 flight control software.
    airsim_port: Network port for AirSim.
    ekf2_port: Network port for Extended Kalman Filter (EKF2).
    px4_port: Network port for PX4.

Paths:
    home_directory: User's home directory. 
    airsim_directory: AirSim environment files. 
    airsim_binary_name: AirSim binary file. 
    px4_directory: Directory for the PX4 firmware. 
    mission_file: Path to the mission file defining the flight plan.
    agent_name: Name assigned to the recovery agent. 

SimulationSettings:
    len_timestep: Length of each simulation timestep.
    movement_factor: Scaling factor for the movement of the drone in the simulation.
    movement_resolution: Resolution of predictive steps.
    detection_delay: Attack detection delay.
    postattack_delay: Time delay to resume normal operations post attack detection.
    buffer_size: Buffer size for storing simulation data.
    attack_cooldown: Time interval between attack instances. 

AttackerSettings:
    attack_type: Target sensor (e.g., gps, or gyro).
    attack_lower_bound: Attack bias lower bound.
    attack_upper_bound: Attack bias upper bound. 
    attack_dur: Attack duration.
    attack_delay: Time interval between attack instances.
```

## Output

Each training or evaluation run will generate a directory under `out/`. The following is the output structure:

```
nh_gps_drone_0-2024_00_00_000000
├── recovery_models
│   ├── recovery_model_1000_steps.zip
│   └── recovery_model_2000_steps.zip
├── attacker_models
│   └── attack_model_200_steps.zip
├── attacker_tb_logs/RecurrentPPO_1
├── recovery_tb_logs/RecurrentPPO_1
├── loaded_recovery_model.zip
├── loaded_attacker_model.zip
├── evaluation.txt
└── config.yaml
```

- `recovery_models`: checkpointed recovery models
- `attacker_models`: checkpointed attacker models
- `recovery_tb_logs`: Tensorboard logs for recovery model
- `attacker_tb_logs`: Tensorboard logs for attacker model
- `loaded_recovery_model.zip`: the trained recovery model 
- `loaded_attacker_model.zip`: the trained attacker model 
- `evaluation.txt`: the evaluation results for the model, if requested
- `config.yaml`: the configuration file used for the training/evaluation


### Tensorboard Logs

To read Tensorboard logs, go to `attacker_tb_logs/` or `recovery_tb_logs/` directory and run:
```
tensorboard --logdir=RecurrentPPO_1
```
The logs can be viewed at:
```
TensorBoard 2.12.3 at http://localhost:6006/
```

### Evaluation

The file `evaluation.txt` shows the results in the following format. 
```
ATTACKS: 100 | RECOVERIES: 94  | COLLISIONS: 0
```

- `ATTACKS`: number of attacks launched
- `RECOVERIES`: successful recoveries
- `COLLISIONS`: collisions with obstacle or ground