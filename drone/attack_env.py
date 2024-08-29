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

SENSOR_SIZE = 21

class Attack():
    """
    Represents an attack in a simulation environment.

    Attributes:
        total_duration (int): The total duration for which the attack is active.
        current_duration (int): The remaining duration of the attack. Initially set to total_duration.
        type (str): The type of attack (e.g., 'gps', 'gyro').
        magnitude (list of float): The magnitude of the attack, defined as a list of float values.

    Methods:
        __str__(): Returns a string representation of the attack object.
    """

    def __init__(self, total_duration, attack_type, magnitude):
        """
        Initializes an Attack object with specified attributes.

        Args:
            total_duration (int): Total duration of the attack.
            attack_type (str): Type of the attack.
            magnitude (list of float): Magnitude of the attack.
        """

        self.total_duration = total_duration  # total duration of the attack
        self.current_duration = total_duration  # initialized to zero, can be updated during the attack
        self.type = attack_type  # type of attack as a string
        self.magnitude = magnitude  # list of double values

    def __str__(self):
        """
        String representation of the Attack object.

        Returns:
            str: A formatted string displaying the attack's properties.
        """

        return f"Attack(type={self.type}, total_duration={self.total_duration}, current_duration={self.current_duration}, magnitude={self.magnitude})"

class AttackEnv(gym.Env):

    def __init__(
        self,
        attack_queue,
        defense_queue,
        reward_queue,
        simulation_settings,
        attacker_settings,
        verbose
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
        self.verbose = verbose

        self.attack_type = attacker_settings["attack_type"]
        self.buffer_size = simulation_settings["buffer_size"]
        self.reward_queue = reward_queue
        self.defense_queue = defense_queue
        self.attack_queue = attack_queue
        self.len_timestep = simulation_settings["len_timestep"]

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.buffer_size, SENSOR_SIZE), dtype=np.float32)

        if (self.attack_type == "gps"):
            self.action_space = spaces.MultiDiscrete([attacker_settings["attack_delay"] + 1, attacker_settings["attack_dur"][1] - attacker_settings["attack_dur"][0] + 1,
                                                         attacker_settings["attack_upper_bound"][0] - attacker_settings["attack_lower_bound"][0] + 1, 
                                                         attacker_settings["attack_upper_bound"][1] - attacker_settings["attack_lower_bound"][1] + 1,
                                                         attacker_settings["attack_upper_bound"][2] - attacker_settings["attack_lower_bound"][2] + 1])
        elif (self.attack_type == "gyro"):
            self.action_space = spaces.MultiDiscrete([attacker_settings["attack_delay"] + 1, attacker_settings["attack_dur"][1] - attacker_settings["attack_dur"][0] + 1,
                                                        3,
                                                         attacker_settings["attack_upper_bound"][0] - attacker_settings["attack_lower_bound"][0] + 1])
        else:
            sys.exit(1)

    def printVerbose(self, message):
        """
        Prints a message if verbose mode is enabled.

        Args:
            message (str): The message to be printed in verbose mode.

        Attributes:
            verbose (bool): A flag indicating whether verbose mode is active.
        """

        # Check if verbose mode is enabled
        if self.verbose:
            # Print the verbose message
            print("VERBOSE: " + str(message))

    def reset(self):
        """
        Resets the environment to its initial state.

        Returns:
            np.array: The initial observation from the defense queue.
        """

        return self._get_obs()

    def _wait_for_reward(self):
        """
        Waits and retrieves the reward from the reward queue.

        Returns:
            float: The reward value obtained from the simulation.
        """

        reward = 0

        while True:
            
            try:
                reward = self.reward_queue.get_nowait()
    
                break
            except Empty:
                pass

            try:
                observation = self.defense_queue.get_nowait()
                reward = 0
                break
            except Empty:
                pass    

            time.sleep(0.01)

        # Flush reward buffer
        for i in range(10):

            try:
                self.reward_queue.get_nowait()
            except Empty:
                pass

            time.sleep(0.05)
           
        return reward

    def _get_obs(self):
        """
        Retrieves the latest observation from the defense queue.

        Returns:
            np.array: The current observation of the environment.
        """

        while True:
            
            try:

                observation = self.defense_queue.get_nowait()
    
                break
            except Empty:
                pass

            time.sleep(0.01)


        # Flush defense buffer
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
        done = True

        time.sleep(action[0])

       
        if (self.attack_type == "gps"):
            new_attack = Attack(action[1] + self.attacker_settings["attack_dur"][0], "gps", 
                                    [action[2] + self.attacker_settings["attack_lower_bound"][0], 
                                         action[3] + self.attacker_settings["attack_lower_bound"][1],
                                         action[4] + self.attacker_settings["attack_lower_bound"][2]])
        elif (self.attack_type == "gyro"):
            new_attack = Attack(action[1] + self.attacker_settings["attack_dur"][0], "gyro", 
                                        [action[2], 
                                         action[3] + self.attacker_settings["attack_lower_bound"][1]])
            
        self.attack_queue.put(new_attack)
        
        # Wait for the duration of the attack
        time.sleep(action[1] + self.attacker_settings["attack_dur"][0])

        # Receive reward and print it
        reward = -1 * self._wait_for_reward()

        self.printVerbose("ATTACKER REWARD: " + str(reward))

        # Prepare a zeroed observation array
        obs = np.zeros(SENSOR_SIZE)
 
        return obs, reward, done, info

