# SpecGuard: Rover

We integrate ArduPilot autopilot with Microsoft AirSim to train Deep-RL recovery control policy for rovers.

## Installation and Setup

1. **Ardupilot**: Follow the instructions for installing and building ArduPilot with AirSim [ArduPilot Docs for AirSim](https://ardupilot.org/dev/docs/sitl-with-airsim.html). **Checkout commit 4dc9d1ebc7405cf261b36ad6fd2a2e4bfd764ec3**, `git checkout 4dc9d1ebc7405cf261b36ad6fd2a2e4bfd764ec3`.
2. **Copy the AirSim Settings`**: Copy ```settings.json``` into the AirSim directory. 
    - `ardupilot_directory`: ArduPilot installation directory
    - `airsim_binary_name`: the name of the Airsim binary 
```
Paths:
  home_directory: /home/YOUR_USER_HERE
  airsim_directory: /airsim-envs/AirSimNH/LinuxNoEditor/
  airsim_binary_name: AirSimNH
  ardupilot_directory: /ardupilot
  mission_file: ./missions/nh_box_3.txt
  agent_name: nh_gps
``` 

## Usage

To run the pipeline, you need to specify a configuration file:
```
python ppo_rover.py -c /path/to/config
```

Ensure the configuration file is properly configured with the paths to your environment binaries and installations.

### Options
  - `-c/--config path/to/config` the path to your configuration file.
  - `-n/--name agent_name` the name of the trained policy.
  - `-lr/--load-recovery` to load a trained policy.
  - `-la/--load-attacker` to load a trained anomaly agent.
  - `-v/--verbose` print verbose output for debugging.
  - `-e/--evaluate num_evaluation_episodes` number of evaluation episodes.

### Examples
```
python ppo_rover.py -c configs/nh_gps.yaml
python ppo_rover.py -c configs/cb_gps.yaml -lr trained_recovery_model.zip
python ppo_rover.py -c configs/nh_gyro.yaml -lr trained_recovery_model.zip -la trained_attacker_model.zip -e 100 -v
```

## Configuration Files

Configuration files specify the environment, attack settings, and training hyperparameters. 

### Parameters

```
Hyperparameters:
    gamma: Discount factor for future rewards in the reinforcement learning algorithm.
    clip_ratio: Used to limit policy updates, ensuring changes aren't too drastic.
    value_function_learning_rate: Learning rate for the value function in the RL model.
    lam: Lambda parameter for Generalized Advantage Estimation (GAE).
    save_freq: Frequency (in timesteps) at which the model is saved.
    train_steps: Total number of training steps to be executed.

Connections:
  airsim_ip: IP address for AirSim.
  ardupilot_ip: IP address for ArduPilot.
  airsim_port: Network port for AirSim.
  ardupilot_port: Network port for ArduPilot.

Paths:
    home_directory: User's home directory. 
    airsim_directory: AirSim environment files. 
    airsim_binary_name: AirSim binary file. 
    ardupilot_directory: Directory for the ArduPilot firmware. 
    mission_file: Path to the mission file defining the flight plan.
    agent_name: Name assigned to the recovery agent. 

SimulationSettings:
  len_timestep: Length of each simulation timestep.
  throttle_magnitude: Scaling factor for rover's throttle.
  steering_magnitude: Scaling factor for the rover's steering.
  throttle_resolution: Resolution for throttle prediction values. 
  steering_resolution: Resolution for steering prediction values. 
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
nh_gps_0-2024_00_00_000000
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

To read Tensorboard logs, navigate into the `attacker_tb_logs/` or `recovery_tb_logs/` directory and run:
```
tensorboard --logdir=RecurrentPPO_1
```
The logs can be viewed at:
```
TensorBoard 2.12.3 at http://localhost:6006/ (Press CTRL+C to quit)
```

### Evaluation

The file `evaluation.txt` shows the results in the following format. 
```
ATTACKS: 100 | RECOVERIES: 93  | COLLISIONS: 0
```

- `ATTACKS`: number of attacks launched
- `RECOVERIES`: successful recoveries
- `COLLISIONS`: collisions with obstacle or ground
```