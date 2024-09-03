"""
## Import Libraries
"""

import gym
from gym import spaces
import numpy as np
import sys
from queue import Empty
import time

"""
## Constants
"""

SENSOR_SIZE = 23  # Constant defining the size of the sensor array.

class Attack():
    def __init__(self, total_duration, attack_type, magnitude):
        """
        Initializes an instance of the Attack class.

        Args:
            total_duration (int): Total duration of the attack in simulation time steps.
            attack_type (str): The type of attack, e.g., 'gps' or 'gyro'.
            magnitude (list of double): Parameters defining the intensity of the attack.
        """
        self.total_duration = total_duration  # total duration of the attack
        self.current_duration = total_duration  # initialized to the same value as total_duration
        self.type = attack_type  # type of attack as a string
        self.magnitude = magnitude  # list of double values defining the attack intensity

    def __str__(self):
        return f"Attack(type={self.type}, total_duration={self.total_duration}, current_duration={self.current_duration}, magnitude={self.magnitude})"


class AttackEnv(gym.Env):
    def __init__(
        self,
        attack_queue,
        defense_queue,
        reward_queue,
        simulation_settings,
        attacker_settings
    ):
        """
        Initializes an instance of the AttackEnv class, a custom gym environment.

        Args:
            attack_queue (queue.Queue): A queue for storing generated attack instances.
            defense_queue (queue.Queue): A queue for observations from the defense mechanism.
            reward_queue (queue.Queue): A queue for receiving rewards based on attack outcomes.
            simulation_settings (dict): A dictionary containing simulation settings such as buffer size and timestep length.
            attacker_settings (dict): A dictionary containing attacker-specific settings such as attack type and duration.
        """
        self.attacker_settings = attacker_settings
        self.attack_type = attacker_settings["attack_type"]
        self.buffer_size = simulation_settings["buffer_size"]
        self.reward_queue = reward_queue
        self.defense_queue = defense_queue
        self.attack_queue = attack_queue
        self.len_timestep = simulation_settings["len_timestep"]

        # Set observation and action space based on simulation and attacker settings
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.buffer_size, SENSOR_SIZE), dtype=np.float32)
        if self.attack_type == "gps":
            self.action_space = spaces.MultiDiscrete([
                attacker_settings["attack_delay"] + 1,
                attacker_settings["attack_dur"][1] - attacker_settings["attack_dur"][0] + 1,
                attacker_settings["attack_upper_bound"][0] - attacker_settings["attack_lower_bound"][0] + 1,
                attacker_settings["attack_upper_bound"][1] - attacker_settings["attack_lower_bound"][1] + 1
            ])
        elif self.attack_type == "gyro":
            self.action_space = spaces.MultiDiscrete([
                attacker_settings["attack_delay"] + 1,
                attacker_settings["attack_dur"][1] - attacker_settings["attack_dur"][0] + 1,
                attacker_settings["attack_upper_bound"][0] - attacker_settings["attack_lower_bound"][0] + 1
            ])
        else:
            sys.exit(1)  # Exit the program if attack type is not recognized

    def reset(self):
        """
        Resets the environment to its initial state and returns the initial observation.

        Returns:
            ndarray: The initial observation of the environment after reset.
        """
        return self._get_obs()  # Fetches and returns the initial observation

    def _wait_for_reward(self):
        """
        Waits and retrieves the reward from the reward queue.

        Returns:
            float: The retrieved reward value.
        """
        reward = 0
        while True:
            try:
                reward = self.reward_queue.get_nowait()  # Attempt to retrieve the reward immediately
                break
            except Empty:
                pass  # If the queue is empty, continue waiting
            time.sleep(0.01)  # Sleep for a short duration before retrying

        
        for i in range(10):

            try:
                self.reward_queue.get_nowait()
            except Empty:
                pass

            time.sleep(0.05)

        return reward

    def _get_obs(self):
        """
        Waits and retrieves the next observation from the defense queue.

        Returns:
            ndarray: The retrieved observation.
        """
        while True:
            try:
                observation = self.defense_queue.get_nowait()  # Attempt to retrieve the observation immediately
                break
            except Empty:
                pass  # If the queue is empty, continue waiting
            time.sleep(0.01)  # Sleep for a short duration before retrying


        for i in range(10):

            try:
                self.defense_queue.get_nowait()
            except Empty:
                pass

            time.sleep(0.05)

        return observation

    def step(self, action):
        """
        Performs the given action in the environment and returns the results.

        Args:
            action (list): The action to be taken in the environment.

        Returns:
            tuple: A tuple containing the new observation, reward, done flag, and info dictionary.
            - obs (ndarray): The new observation after performing the action.
            - reward (float): The computed reward as a result of the action.
            - done (bool): Flag indicating if the episode (attack instance) has ended.
            - info (dict): Additional information about the step.
        """
        info = {}
        done = True  # Assumes the episode ends after each step in this environment

        # Delay action execution by the specified amount
        time.sleep(action[0])

        # Create a new attack instance based on the action and enqueue it
        if self.attack_type == "gps":
            new_attack = Attack(
                action[1] + self.attacker_settings["attack_dur"][0],
                "gps",
                [action[2] + self.attacker_settings["attack_lower_bound"][0],
                 action[3] + self.attacker_settings["attack_lower_bound"][1]]
            )
        elif self.attack_type == "gyro":
            new_attack = Attack(
                action[1] + self.attacker_settings["attack_dur"][0],
                "gyro",
                [action[2] + self.attacker_settings["attack_lower_bound"][0]]
            )
        self.attack_queue.put(new_attack)

        # Wait for the duration of the attack
        time.sleep(action[1] + self.attacker_settings["attack_dur"][0])

        # Receive reward and print it
        reward = -1 * self._wait_for_reward()


        # Prepare a zeroed observation array
        obs = np.zeros(SENSOR_SIZE)

        return obs, reward, done, info
