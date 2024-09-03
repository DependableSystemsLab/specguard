"""
## Import Libraries
"""

import yaml
import argparse
import queue
import threading
from sb3_contrib import RecurrentPPO
from stable_baselines3.common.callbacks import CheckpointCallback
from datetime import datetime
from recovery_env import RecoveryEnv
from attack_env import AttackEnv
import argparse
import numpy as np
import os
import sys

"""
## Helper Functions
"""

def extract_config_with_defaults(config, section_name, default_values):

    section_config = config.get(section_name, {})
    extracted_values = {key: section_config.get(key, default) for key, default in default_values.items()}

    return extracted_values

def extract_attacker_settings(config):
    # Define the default values for the attacker settings
    attacker_settings_defaults = {
        'attack_type': 'gps',
        'attack_lower_bound': [5, 5],
        'attack_upper_bound': [5, 5],
        'attack_dur': [5, 20],
        'attack_delay': 5
    }
    
    # Extract the AttackerSettings section using the default values
    attacker_settings = extract_config_with_defaults(config=config, section_name="AttackerSettings", default_values=attacker_settings_defaults)
    
    return attacker_settings

def write_config(file_path, config_data):
    with open(file_path, 'w') as file:
        yaml.dump(config_data, file, default_flow_style=False)


"""
## Get Command Line Arguments
"""

# Initalize argparser
parser = argparse.ArgumentParser(description="Configure the simulation")

# Add mandatory argument
parser.add_argument('-c', '--config', required=True, help='Path to the configuration YAML file.')

# Add optional argument
parser.add_argument('-d', '--docker', action='store_true', default=False, help='Must be set if simulation is running in Docker.')
parser.add_argument('-v', '--verbose', action='store_true', default=False, help='Print verbose output to console for debugging.')
parser.add_argument('-e', '--evaluate', type=int, default=0, help='Number of evaluation episodes to run.')
parser.add_argument('-n', '--name', type=str, default='', help='Name of the agent, which overrides the default if provided.')
parser.add_argument('-lr', '--load-recovery', type=str, default='', help='Path to the recovery model that should be loaded.')
parser.add_argument('-la', '--load-attacker', type=str, default='', help='Path to the attack model that should be loaded.')


# Parse arguments
args = parser.parse_args()

# Parse arguments to variables
docker = args.docker
verbose = args.verbose
evaluate = args.evaluate

env_model_path = args.load_recovery
attacker_model_path = args.load_attacker


"""
## Read Configuration File
"""

# Open the configuration file
with open(args.config, 'r') as file:
    config = yaml.safe_load(file)

# Default values
hyperparameter_defaults = {
    'gamma': 0.90,
    'clip_ratio': 0.2,
    'value_function_learning_rate': 0.001,
    'lam': 0.97,
    'save_freq': 1000,
    'train_steps': 1000000
}

simulation_settings_defaults = {
    'len_timestep': 1,
    'throttle_magnitude': 25,
    'steering_magnitude': 4500,
    'throttle_resolution': 1,
    'steering_resolution': 5,
    'detection_delay': 2,
    'postattack_delay': 2,
    'buffer_size': 10,
    'attack_cooldown': 5
}

paths_defaults = {
    'airsim_directory': '/airsim-envs/AirSimNH/LinuxNoEditor/',
    'airsim_binary_name': 'AirSimNH',
    'ardupilot_directory': '/ardupilot',
    'mission_file': '/missions/box_waypoints.txt',
    'agent_name': 'nh_default'
}

connections_defaults = {
    'airsim_ip': '127.0.0.1',
    'ardupilot_ip': '127.0.0.1',
    'airsim_port': 14550,
    'ardupilot_port': 14551
}


# Extract config file
hyperparameters = extract_config_with_defaults(config=config, section_name="Hyperparameters", default_values=hyperparameter_defaults)
simulation_settings = extract_config_with_defaults(config=config, section_name="SimulationSettings", default_values=simulation_settings_defaults)
connections = extract_config_with_defaults(config=config, section_name="Connections", default_values=connections_defaults)
paths = extract_config_with_defaults(config=config, section_name="Paths", default_values=paths_defaults)
attacker_settings = extract_attacker_settings(config)

if (docker):
    paths["ardupilot_directory"] = "/home/airsim_user/ardupilot"
    paths["airsim_directory"] = "/home/airsim_user/environments/AirSimNH/LinuxNoEditor/"

"""
## Attack/Defense Communication
"""

# Shared state variables
attack_queue = queue.Queue()
defense_queue = queue.Queue()
reward_queue = queue.Queue()

"""
## Initializing the agents
"""


agent_name = args.name if args.name else paths['agent_name']


now = datetime.now()
    
base_dir = 'out/'

os.makedirs(base_dir, exist_ok=True)

existing_instances = len([name for name in os.listdir(base_dir) if name.startswith(agent_name)]) 
logdir = f'{base_dir}{agent_name}_{existing_instances}-{now.strftime("%Y_%m_%d_%H%M%S")}'

# Make directories to save
os.makedirs(logdir, exist_ok=True)

# Make sub directories
os.makedirs(f'{logdir}/recovery_tb_logs', exist_ok=True)
os.makedirs(f'{logdir}/attacker_tb_logs', exist_ok=True)
os.makedirs(f'{logdir}/recovery_models', exist_ok=True)
os.makedirs(f'{logdir}/attacker_models', exist_ok=True)

# Write config
write_config(os.path.join(logdir, 'config.yaml'), config)

# Make environment
env = RecoveryEnv(simulation_settings=simulation_settings,
                    paths=paths,
                    connections=connections,
                    attack_queue=attack_queue,
                    defense_queue=defense_queue,
                    reward_queue=reward_queue,
                    docker=docker,
                    verbose=verbose,
                    evaluate=evaluate,
                    logdir=logdir
                    )

# Make attacker
attacker = AttackEnv(simulation_settings=simulation_settings,
                    attack_queue=attack_queue,
                    defense_queue=defense_queue,
                    reward_queue=reward_queue,
                    attacker_settings=attacker_settings)




def train_defense_model(logdir, model_path, hyperparameters):
    # Save a checkpoint every save_freq steps
    checkpoint_callback = CheckpointCallback(save_freq=hyperparameters["save_freq"], save_path=f'{logdir}/recovery_models/', name_prefix='recovery_model')

    # Check if a model path is provided and try loading the model
    if model_path:
        try:
            model = RecurrentPPO.load(model_path, env, tensorboard_log=f"{logdir}/recovery_tb_logs/")
            print("Successfully loaded model from:", model_path)

            model.save(f'{logdir}/loaded_recovery_model')
        except Exception as e:
            print("Error loading model:", e)
            sys.exit(1)
    else:
        model = RecurrentPPO("MlpLstmPolicy", env, verbose=1, gamma=hyperparameters["gamma"], gae_lambda=hyperparameters["lam"], clip_range=hyperparameters["clip_ratio"], learning_rate=hyperparameter_defaults["value_function_learning_rate"], tensorboard_log=f"{logdir}/recovery_tb_logs/")

    # Create and train the model
    model.learn(total_timesteps=hyperparameters["train_steps"], callback=[checkpoint_callback])

def defense_training_thread(logdir, model_path, hyperparameters):
    defense_training_thread = threading.Thread(target=train_defense_model, args=(logdir, model_path, hyperparameters))
    defense_training_thread.start()
    return defense_training_thread

def train_attacker_model(logdir, model_path, hyperparameters):
    # Save a checkpoint every save_freq steps
    attack_checkpoint_callback = CheckpointCallback(save_freq=50, save_path=f'{logdir}/attacker_models/', name_prefix='attack_model')

    if (model_path):
        try:
            attack_model = RecurrentPPO.load(model_path, attacker, tensorboard_log=f"{logdir}/attacker_tb_logs/")
            print("Successfully loaded model from:", model_path)

            attack_model.save(f'{logdir}/loaded_attacker_model')
        except Exception as e:
            print("Error loading model:", e)
            sys.exit(1)
    else:
        attack_model = RecurrentPPO("MlpLstmPolicy", attacker, verbose=1, gamma=hyperparameters["gamma"], gae_lambda=hyperparameters["lam"], clip_range=hyperparameters["clip_ratio"], learning_rate=hyperparameter_defaults["value_function_learning_rate"], tensorboard_log=f"{logdir}/attacker_tb_logs/")

    # Create and train the model
    attack_model.learn(total_timesteps=hyperparameters["train_steps"], callback=[attack_checkpoint_callback])


def attacker_training_thread(logdir, model_path, hyperparameters):
    attacker_training_thread = threading.Thread(target=train_attacker_model, args=(logdir, model_path, hyperparameters))
    attacker_training_thread.start()
    return attacker_training_thread


def evaluate_defense_model(logdir, model_path, num_evaluation_episodes):

    try:
        model = RecurrentPPO.load(model_path)
        print("Successfully loaded model from:", model_path)
        model.save(f'{logdir}/loaded_env_model')
    except Exception as e:
        print("Error loading model:", e)
        sys.exit(1)
    
    for episode in range(num_evaluation_episodes):
        obs = env.reset()
        done = False

        while not done:
            action, _ = model.predict(obs)
            obs, reward, done, info = env.step(action)
        
def defense_evaluation_thread(logdir, model_path, num_evaluation_episodes):
    defense_evaluation_thread = threading.Thread(target=evaluate_defense_model, args=(logdir, model_path, num_evaluation_episodes))
    defense_evaluation_thread.start()
    return defense_evaluation_thread

def attacker_evaluation_thread(logdir, model_path, num_evaluation_episodes):
    attacker_evaluation_thread = threading.Thread(target=evaluate_attacker_model, args=(logdir, model_path, num_evaluation_episodes))
    attacker_evaluation_thread.start()
    return attacker_evaluation_thread

def evaluate_attacker_model(logdir, model_path, num_evaluation_episodes):
    try:
        attack_model = RecurrentPPO.load(model_path)
        print("Successfully loaded model from:", model_path)
        attack_model.save(f'{logdir}/loaded_attacker_model')
    except Exception as e:
        print("Error loading model:", e)
        attack_model = RecurrentPPO("MlpLstmPolicy", attacker, verbose=1, gamma=hyperparameters["gamma"], gae_lambda=hyperparameters["lam"], clip_range=hyperparameters["clip_ratio"], learning_rate=hyperparameter_defaults["value_function_learning_rate"], tensorboard_log=f"{logdir}/attacker_tb_logs/")
    
    for num_evaluation_episodes in range(num_evaluation_episodes * 2):
        obs = attacker.reset()
        done = False

        while not done:
            action, _ = attack_model.predict(obs)
            obs, reward, done, info = attacker.step(action)
   
# Check if running in evaluation mode
if args.evaluate > 0:

    time.sleep(20)

    # Evaluation mode: run evaluation threads for both defense and attack agents
    defense_thread = defense_evaluation_thread(logdir, args.load_recovery, args.evaluate)
    attacker_thread = attacker_evaluation_thread(logdir, args.load_attacker, args.evaluate)
else:
    # Training mode: start training threads for both defense and attack agents
    defense_thread = defense_training_thread(logdir, args.load_recovery, hyperparameters)
    attacker_thread = attacker_training_thread(logdir, args.load_attacker, hyperparameters)