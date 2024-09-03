"""
## Import Libraries
"""

import math
import typing
import logging
import subprocess
import os
from PIL import Image
import gym
from gym import spaces
from collections import deque


import airsim
import signal
import numpy as np

import random
from pyproj import Proj, Transformer
import socket
import concurrent.futures

import threading
from threading import Timer

from dronekit import connect, APIException
from dronekit import connect, Command, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil, mavwp
import time, sys, argparse, math
import queue

from utils.EuclideanDistance import *
from utils.RoverStatusListener import *

from dronekit import connect

"""
## Constants
"""

SENSOR_SIZE = 23
GYRO_SPOOFING_MAG = 1.047197551
SHORT_SLEEP_1 = 1
SHORT_SLEEP_2 = 2
MEDIUM_SLEEP_1 = 5
MEDIUM_SLEEP_2 = 10
LONG_SLEEP = 100
RECOVERY_THRESHOLD = 2.0
MAXIMUM_TRIES = 3
RECOVERED_TIME = 5


class RecoveryEnv(gym.Env):

    metadata = {"render.modes": ["rgb_array"]}

    def __init__(
        self,
        connections,
        simulation_settings,
        paths,
        attack_queue,
        defense_queue,
        reward_queue,
        docker,
        verbose,
        evaluate,
        logdir
    ):  
        
        """
        A custom Gym environment for an AirSim car simulation.

        Args:
            connections (dict): Contains connection settings for AirSim and ArduPilot.
            simulation_settings (dict): Configuration settings for the simulation such as buffer size, timestep length, etc.
            paths (dict): File paths necessary for the simulation, like directories for AirSim and ArduPilot, mission files, etc.
            attack_queue (queue.Queue): A queue to manage attack actions in the simulation.
            defense_queue (queue.Queue): A queue to manage defense actions in the simulation.
            reward_queue (queue.Queue): A queue to manage rewards within the simulation environment.
            docker (bool): Flag to indicate if the simulation is running in a Docker environment.
            verbose (bool): Enable verbose logging if True.
            evaluate (bool): Flag to indicate if the simulation is in evaluation mode.
            logdir (str): Directory path for logging simulation data.

        Attributes:
            observation_space (spaces.Box): The space of observable states in the environment.
            action_space (spaces.MultiDiscrete): The space of possible actions that can be taken in the environment.
            current_wp (int): Current waypoint index in the mission.
            waypoints (list): List of waypoints in the mission.
            home (object): Home position in the simulation environment.
            full_reset_flag (bool): Flag to indicate if a full reset of the simulation is required.
            attack_reward (float): Accumulated reward during an attack.
            attack_count (int): Count of attacks that have occurred.
            saftey_policy_collisions (int): Count of safety policy collisions.
            saftey_policy_recoveries (int): Count of recoveries from safety policy violations.
            saftey_policy_hit (bool): Flag indicating if a safety policy was hit.
            car (airsim.CarClient): AirSim car client for the simulation.
            master (mavutil.mavlink_connection): MAVLink connection for the simulation.
            listener (RoverStatusListener): Listener for rover status updates.
            wp_listener (WaypointListener): Listener for waypoint updates.
        """
        
        # init environment
        super(RecoveryEnv, self).__init__()

        # unpack simulation settings
        self.buffer_size = simulation_settings["buffer_size"]
        self.len_timestep = simulation_settings["len_timestep"]
        self.throttle_resolution = simulation_settings["throttle_resolution"]
        self.steering_resolution = simulation_settings["steering_resolution"]
        self.throttle_magnitude = simulation_settings["throttle_magnitude"]
        self.steering_magnitude = simulation_settings["steering_magnitude"]
        self.detection_delay = simulation_settings["detection_delay"]
        self.postattack_delay = simulation_settings["postattack_delay"]
        self.attack_cooldown = simulation_settings["attack_cooldown"]
        
        self.evaluate = evaluate
        self.logdir = logdir

        # Queues
        self.defense_queue = defense_queue
        self.reward_queue = reward_queue
        self.attack_queue = attack_queue

        # set if verbose or not
        self.verbose = verbose

        # unpack paths
        self.docker = docker
        
        if (self.docker):
            self.printVerbose("DOCKER FLAG IS SET")

        self.airsim_directory = paths["airsim_directory"]
        self.airsim_binary_name = paths["airsim_binary_name"]
        self.ardupilot_directory = paths["ardupilot_directory"]
        self.mission_file = paths["mission_file"]

        # set observation and action space
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.buffer_size, SENSOR_SIZE), dtype=np.float32)
        self.action_space = spaces.MultiDiscrete([1 + 2 * self.throttle_resolution, 1 + 2 * self.steering_resolution])

        

        # save cwd
        self.cwd = os.getcwd()
        
        # state variables
        self.current_wp = 0
        self.waypoints = []
        self.home = None
        self.full_reset_flag = False
        self.attack_reward = 0

        # Evaluation variables
        self.attack_count = 0
        self.saftey_policy_collisions = 0
        self.saftey_policy_recoveries = 0
    
        # unpack connections
        self.airsim_ip_address = connections["airsim_ip"]
        self.ardupilot_override_ip = connections["ardupilot_ip"]
        self.ardupilot_override_port = connections["ardupilot_port"]
        self.airsim_port = connections["airsim_port"]

        # signal handler
        def handler(signum, frame):
            raise Exception("Timeout")

        # Set the signal handler
        signal.signal(signal.SIGALRM, handler)

        if (self.docker):
            self.prepare_docker_build()

        # cleanup any open processes
        self.exit_ardupilot()
        time.sleep(SHORT_SLEEP_2)
        self.exit_airsim()
        time.sleep(SHORT_SLEEP_2) #nominal exit delay
        
        # launch Ardupilot
        self.launchArdupilot()
        time.sleep(MEDIUM_SLEEP_1)

        # launch airsim
        self.launchAirsim()
        time.sleep(MEDIUM_SLEEP_1)
       
        # airsim api
        self.car = airsim.CarClient(ip=self.airsim_ip_address)
        self.printVerbose("Connecting to the AirSim api...")

        time.sleep(MEDIUM_SLEEP_2)

        # mav link ground control
        self.master = mavutil.mavlink_connection('udp:' + str(self.airsim_ip_address) + ':' + str(self.airsim_port))
    
        # Get home position
        self.getHomePosition()

        self.saftey_policy_hit = False

        self.listener = RoverStatusListener(self.master) 
        self.listener.start()

        self.wp_listener = WaypointListener(self.master)
        self.wp_listener.start()

        self.resetWorkingVars()

        self.done_flag = True

    def prepare_docker_build(self):
        
        self.launchArdupilot()

        time.sleep(LONG_SLEEP)

    
    def cleanup_processes(self):
        """
        Cleans up and terminates all running processes related to the simulation.
        """

        # Stop listener threads
        self.listener.keep_running = False
        self.wp_listener.keep_running = False

        # Exit Ardupilot and AirSim
        self.exit_ardupilot()
        time.sleep(MEDIUM_SLEEP_1)
        self.exit_airsim()
        time.sleep(MEDIUM_SLEEP_1)

        # Close MAVLink connection
        self.master.close()

    def startup_processes(self):
        """
        Starts up all necessary processes for the simulation environment.
        """

        # Launch Ardupilot and Airsim
        self.launchArdupilot()
        time.sleep(MEDIUM_SLEEP_1)
        self.launchAirsim()
        time.sleep(MEDIUM_SLEEP_1)

        # Establish AirSim API connection
        self.airsim_ip_address = "127.0.0.1"
        self.car = airsim.CarClient(ip=self.airsim_ip_address)
        self.printVerbose("Connecting to the AirSim api...")
        time.sleep(MEDIUM_SLEEP_2)

        # Initialize MAVLink ground control
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.listener = RoverStatusListener(self.master) 
        self.listener.start()
        self.wp_listener = WaypointListener(self.master)
        self.wp_listener.start()

    
    def full_reset_ardupilot_airsim(self):
        """
        Performs a full reset of both Ardupilot and AirSim processes.

        This function is used to reset the simulation environment by terminating and restarting the Ardupilot and AirSim processes.
        """
        self.printVerbose("Cleaning up processes...")
        self.cleanup_processes()

        self.printVerbose("Starting up processes...")
        self.startup_processes()

    def launchArdupilot(self):
        """
        Launches the Ardupilot ground controller.

        Args:
            None
        """
        os.chdir(self.ardupilot_directory)

        process_command = [
            "sim_vehicle.py", "-v", "Rover", "-f", "airsim-rover", "--console", "--add-param-file=rover.parm", "-w"
        ]

        if self.docker:
            process_command = ["python3", "./Tools/autotest/sim_vehicle.py", "-v", "Rover", "-f", "airsim-rover", "--add-param-file=rover.parm", "-w"]
            
        
        self.ardupilot_process = subprocess.Popen(process_command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.printVerbose("Started Ardupilot Ground Controller")
        os.chdir(self.cwd)

    def launchAirsim(self):
        """
        Launches the AirSim environment.

        Args:
            None
        """
        command = None

        if self.docker:
            command = ["./" + str(self.airsim_binary_name) + ".sh", "-RenderOffscreen", "-nosound", "-ResX=640", "-ResY=640", "-windowed"]
        else:
            command = ["gnome-terminal", "--", "./" + str(self.airsim_binary_name) + ".sh", "-ResX=640", "-ResY=640", "-windowed"]
            
        self.airsim_process = subprocess.Popen(command, cwd=self.airsim_directory)
        self.printVerbose("Started AirSim")
        os.chdir(self.cwd)

    def printVerbose(self, message):
        """
        Prints a verbose message if verbose mode is enabled.

        Args:
            message (str): The message to be printed in verbose mode.
        """
        if self.verbose:
            print("VERBOSE: " + str(message))

    def exit_airsim(self):
        """
        Terminates the AirSim process if it is running.

        Args:
            None
        """
        self.printVerbose("KILLING AirSim IF OPEN")
        self.kill_process_by_name(self.airsim_binary_name)

    def kill_process_by_name(self, name):
        """
        Kills all processes with a given name.

        Args:
            name (str): The name of the process to be terminated.
        """
        try:
            pid_bytes = subprocess.check_output(["pidof", name])
            pid_string = pid_bytes.decode('utf-8')
            pids = pid_string.split()
            for pid in pids:
                os.kill(int(pid), signal.SIGKILL)
        except:
            pass

    def exit_ardupilot(self):
        """
        Terminates the Ardupilot process if it is running.

        Args:
            None
        """
        self.printVerbose("KILLING ARDUPILOT IF OPEN")
        self.kill_process_by_name("ardurover")
        self.kill_process_by_name("mavproxy")

    def wait_for_attack(self):
        """
        Waits for an attack command and initiates the corresponding attack.

        Args:
            None

        Returns:
            bool: True if an attack is successfully initiated, False otherwise.
        """
        self.current_attack = None
        self.use_autopilot()
        self._flush_obs_queue()

        for i in range(self.attack_cooldown):
            self._compute_reward()
            if self.done_flag:
                return False
            time.sleep(SHORT_SLEEP_1)

        self.attack_reward = 0
        self.recovered_time = 0
        self.saftey_policy_hit = False

        while True:
            self._compute_reward()
            if self.done_flag:
                return False
            obs = self._get_obs()
            self.defense_queue.put(obs)
            try:
                self.current_attack = self.attack_queue.get_nowait()
                self.attack_count += 1
                break
            except queue.Empty:
                pass
            time.sleep(self.len_timestep)

        self._flush_attack_queue()

        if not self.done_flag and self.current_attack is not None:
            self._flush_obs_queue()
            if self.current_attack.type == "gps":
                self.start_gps_attack(self.current_attack.magnitude)
            elif self.current_attack.type == "gyro":
                self.start_gyro_attack(self.current_attack.magnitude)
            time.sleep(self.detection_delay)
            self.current_attack.current_duration = max(1, self.current_attack.current_duration - self.detection_delay)
            return True
        return False

    def _flush_attack_queue(self):
        """
        Clears all items from the attack queue.
        """
        while not self.attack_queue.empty():
            _ = self.attack_queue.get()


    def progress_attack(self):
        """
        Progresses the current attack and checks for recovery.

        Tracks the recovery time post-attack and manages the attack's lifecycle, including stopping the attack when its
        duration ends.

        Returns:
            bool: True if the attack has completed, False if it's still ongoing.
        """
        # Track recovery time
        if self.current_distance_to_path <= RECOVERY_THRESHOLD:
            self.recovered_time += 1
        else:
            self.recovered_time = 0

        # Check if attack duration has ended
        if self.current_attack.current_duration == 0:
            if self.current_attack.type == "gps":
                self.stop_gps_attack()
            elif self.current_attack.type == "gyro":
                self.stop_gyro_attack()

            self.current_attack.current_duration = 0

        # Check if attack and recovery period has ended
        if self.current_attack.current_duration <= -self.postattack_delay:
            if self.current_attack.type == "gps":
                self.stop_gps_attack()
            elif self.current_attack.type == "gyro":
                self.stop_gyro_attack()

            if self.recovered_time >= 5:
                self.saftey_policy_recoveries += 1

            self.current_attack = None
            return True
        else:
            self.current_attack.current_duration -= 1

        return False

    def step(self, action):
        """
        Performs a simulation step given an action.

        Args:
            action: The action to be performed in the simulation.

        Returns:
            tuple: A tuple containing the new observation, reward, done flag, and info dictionary.
        """
        info = {}
        done = False
        
        self._do_action(action)
    

        reward = self._compute_reward()
        self.printVerbose(f"Reward: {reward}")

        self.attack_reward += reward

        attack_done_flag = self.progress_attack()

        obs = self._get_obs()
        
        if attack_done_flag or self.done_flag:
            done = True
            self.reward_queue.put(self.attack_reward)
            self.use_autopilot()

        return obs, reward, done, info

    def confirm_ardupilot_server(self):
        """
        Confirms if the ArduPilot server is available.

        Returns:
            bool: True if the server can be communicated with, False otherwise.
        """
        try:
            self._send_command_to_server("garbage")
            return True
        except Exception as e:
            return False


    def _flush_obs_queue(self):
        """
        Clears all items from the defense queue.
        """
        while not self.defense_queue.empty():
            _ = self.defense_queue.get()

    def _flush_attack_queue(self):
        """
        Clears all items from the attack queue.
        """
        while not self.attack_queue.empty():
            _ = self.attack_queue.get()

    def _flush_queues(self):
        """
        Clears all items from both the attack and reward queues.
        """
        while not self.attack_queue.empty():
            _ = self.attack_queue.get()

        while not self.reward_queue.empty():
            _ = self.reward_queue.get()

    def update_evaluation_file(self, content):
        """
        Updates the evaluation file with the given content.

        Args:
            content (str): Content to be written to the evaluation file.
        """
        if self.evaluate > 0:
            with open(os.path.join(self.logdir, 'evaluation.txt'), 'a') as file:
                file.write(content + '\n')

    def reset(self):
        """
        Resets the environment for a new simulation episode.

        Returns:
            The initial observation after resetting.
        """
        content = f"ATTACKS: {self.attack_count} | RECOVERIES: {self.saftey_policy_recoveries} | COLLISIONS: {self.saftey_policy_collisions}"
        self.update_evaluation_file(content)

        if self.done_flag:
            self._setup_drive()
        
        while not self.wait_for_attack():
            self._setup_drive()

        return self._get_obs()

    def _kill_ardupilot_server(self):
        """
        Attempts to terminate the ArduPilot server.
        """
        try:
            self._send_command_to_server("terminate")
            self.override_mode = False
        except Exception as e:
            self.printVerbose(f"Error killing override SERVER")

    def start_gps_attack(self, magnitude):
        """
        Initiates a GPS spoofing attack with the given magnitude.

        Args:
            magnitude (tuple): The magnitude of the GPS attack.
        """
        try:
            self._send_command_to_server(f"gps start {magnitude[0]} {magnitude[1]}")
        except Exception as e:
            self.printVerbose(f"Error starting GPS Spoofing")

    def stop_gps_attack(self):
        """
        Stops the ongoing GPS spoofing attack.
        """
        try:
            self._send_command_to_server("gps stop")
        except Exception as e:
            self.printVerbose(f"Error stopping GPS Spoofing")

    def start_gyro_attack(self, magnitude):
        """
        Initiates a gyro spoofing attack with the given magnitude.

        Args:
            magnitude (float): The magnitude of the gyro attack.
        """
        try:
            self._send_command_to_server(f"gyro start {magnitude[0] * GYRO_SPOOFING_MAG}")
        except Exception as e:
            self.printVerbose(f"Error starting GYRO Spoofing")

    def stop_gyro_attack(self):
        """
        Stops the ongoing gyro spoofing attack.
        """
        try:
            self._send_command_to_server("gyro stop")
        except Exception as e:
            self.printVerbose(f"Error stopping GYRO Spoofing")

    

    def rebootEKF2(self):
        """
        Resets the Extended Kalman Filter 2 (EKF2) of the flight control system.
        """
        self.printVerbose("Resetting EKF2...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command id
            0,  # confirmation
            0,  # param1 (not used)
            0,  # param2 (not used)
            0,  # param3 (not used)
            0,  # param4 (not used)
            1,  # param5 (reset EKF/INS)
            0,  # param6 (not used)
            0   # param7 (not used)
        )

    def rebootArduPilot(self):
        """
        Reboots the ArduPilot flight control system.
        """
        self.printVerbose("Rebooting ArduPilot...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command id
            0,  # confirmation
            1,  # reboot autopilot
            0,  # param2 (not used)
            0,  # param3 (not used)
            0,  # param4 (not used)
            0,  # param5 (not used)
            0,  # param6 (not used)
            0   # param7 (not used)
        )

    def setAutoMode(self):
        """
        Sets the vehicle to AUTO mode.

        Returns:
            bool: True if the vehicle successfully switches to AUTO mode, False otherwise.
        """
        self.printVerbose("Switching to AUTO Mode")

        for i in range(MAXIMUM_TRIES):
            # Send request to transition to AUTO mode
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.master.mode_mapping()['AUTO']
            )

            time.sleep(3)

            # Check if the transition was successful
            successful = self.timeout_check_listener("current_mode", 5, ["AUTO"], False)

            if successful:
                return True

        return False

    def waitSystemStatus(self):
        """
        Waits until the system status indicates that the system is ready.

        Returns:
            bool: True if the system is ready within the specified timeout, False otherwise.
        """
        self.printVerbose("Waiting for the system to be ready")
        return self.timeout_check_listener("system_status", 20, [3, 4], False)


    def uploadMission(self, filename):
        """
        Uploads a mission from a file to the flight controller.

        Args:
            filename (str): Path to the file containing the mission waypoints.

        Returns:
            bool: True if the mission is successfully uploaded.
        """
        wp = mavwp.MAVWPLoader()
        wp.load(filename)
        self.printVerbose(f"Mission loaded with {wp.count()} waypoints")

        self.printVerbose("Clearing existing missions...")
        self.master.waypoint_clear_all_send()
        self.printVerbose("Existing missions cleared")

        self.printVerbose("Starting mission upload...")
        self.master.waypoint_count_send(wp.count())

        first_waypoint = None
        self.waypoints = []

        self.printVerbose("Uploading mission...")
        for i in range(wp.count()):
            waypoint = wp.wp(i)

            if first_waypoint is None:
                first_waypoint = LocationGlobalRelative(waypoint.x, waypoint.y, waypoint.z)

            waypoint.x = (waypoint.x * 1e7) - (first_waypoint.lat * 1e7) + self.home.latitude
            waypoint.y = (waypoint.y * 1e7) - (first_waypoint.lon * 1e7) + self.home.longitude

            self.waypoints.append([waypoint.x / 1e7, waypoint.y / 1e7, waypoint.z])

            self.master.mav.mission_item_int_send(
                self.master.target_system, self.master.target_component, i, waypoint.frame,
                waypoint.command, 0, 0, waypoint.param1, waypoint.param2, waypoint.param3,
                waypoint.param4, int(waypoint.x), int(waypoint.y), waypoint.z
            )
            time.sleep(0.05)

        return True

    def startMission(self):
        """
        Starts the uploaded mission.

        Returns:
            bool: True if the mission is successfully started.
        """
        self.printVerbose("Starting mission...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        return True

    def resetWorkingVars(self):
        """
        Resets the working variables to their initial state.
        """
        # Reset waypoint, flag, observation, state, position, and attack variables
        self.current_wp = 0
        self.waypoints = []
        self.override_mode = False
        self.done_flag = False
        self.observation_buffer = deque(maxlen=self.buffer_size)
        self.past_position = Point3D(0, 0, 0)
        self.current_position = Point3D(0, 0, 0)
        self.next_waypoint_position = Point3D(0, 0, 0)
        self.past_waypoint_position = Point3D(0, 0, 0)
        self.past_distance_to_current_wp = 0
        self.current_distance_to_current_wp = 0
        self.past_distance_to_path = 0
        self.current_distance_to_path = 0
        self.current_attack = None
        self.stalling_count = 0
        self.attack_reward = 0

        # Get current GPS position
        try:
            carState = self.car.getCarState()
            carEstimatedPosition = carState.kinematics_estimated.position
            self.current_gps_position = Point3D(carEstimatedPosition.x_val, carEstimatedPosition.y_val, carEstimatedPosition.z_val)
            self.past_gps_position = Point3D(carEstimatedPosition.x_val, carEstimatedPosition.y_val, carEstimatedPosition.z_val)
        except:
            self.full_reset_flag = True
            return None
   
    
    def airsimResetTimeout(self, timeout=5):
        """
        Attempts to reset the AirSim environment with a specified timeout.

        Args:
            timeout (int): The number of seconds to wait before considering the reset operation a failure.

        Returns:
            bool: True if the reset is successful within the timeout, False otherwise.
        """
        # Wrapper function to call the reset method
        def wrapper():
            self.car.reset()

        # Start the reset operation in a separate thread
        thread = threading.Thread(target=wrapper)
        thread.start()
        thread.join(timeout)

        # Check if the thread is still alive after the timeout
        if thread.is_alive():
            # Reset function timed out
            return False
        else:
            # Reset was successful
            return True
        

    def _setup_drive(self):
        """
        Sets up the vehicle for driving, including arming, positioning, and starting the mission.
        """

        self._flush_obs_queue()
        self._flush_attack_queue()
        self.reward_queue.put(self.attack_reward)

        current_attempts = -1

        while True:
            current_attempts += 1

            # Reset working variables
            self.resetWorkingVars()

            if self.full_reset_flag or current_attempts >= MAXIMUM_TRIES:
                self.full_reset_ardupilot_airsim()
                self.full_reset_flag = False
                current_attempts = -1
                continue

            # Disarm the vehicle
            self.master.arducopter_disarm()

            # Kill the ArduPilot server and wait
            self._kill_ardupilot_server()
            time.sleep(MEDIUM_SLEEP_1)

            # Reset the car's position
            if not self.airsimResetTimeout():
                self.printVerbose("CAR DID NOT CONNECT")
                continue

            # Reboot EKF2 and ArduPilot and wait
            self.rebootEKF2()
            self.rebootArduPilot()
            time.sleep(MEDIUM_SLEEP_1)

            # Check if the system is ready
            self.listener.system_status = None
            if not self.waitSystemStatus():
                continue

            self.printVerbose("System is now ready.")

            # Ensure GPS has a good fix
            self.listener.gps_status = 0
            if not self.timeout_check_listener("gps_status", 20, [3, 4, 5, 6, 7, 8], False):
                continue

            self.printVerbose("GPS is good")

            # Upload the mission
            if not self.uploadMission(self.mission_file):
                continue

            self.printVerbose("Mission has been successfully uploaded.")

            # Ensure EKF is healthy
            self.listener.ekf_healthy = False
            if not self.timeout_check_listener("ekf_healthy", 60, [True], False):
                continue

            self.printVerbose("Transitioning into AUTO mode")

            # Set to AUTO mode
            self.listener.current_mode = None
            if not self.setAutoMode():
                continue

            self.printVerbose("AUTO mode has been set")

            # Arm the vehicle
            self.listener.armed = False
            if not self.armCar():
                continue

            self.printVerbose("Vehicle has been successfully armed.")

            # Confirm ArduPilot server is running
            if not self.confirm_ardupilot_server():
                continue

            # Start the mission
            if not self.startMission():
                continue

            # Flush the queues
            self._flush_queues()

            break

        self.printVerbose("Escaped the reboot sequence.")


    def timeout_check_listener(self, attr_name, timeout, possible_values, not_none):
        """
        Checks an attribute of the listener within a specified timeout.

        Args:
            attr_name (str): The name of the attribute to check.
            timeout (int): The maximum time to wait in seconds.
            possible_values (list): A list of values to check against the attribute.
            not_none (bool): If True, also checks that the attribute is not None.

        Returns:
            bool: True if the attribute matches any of the possible values or is not None (based on not_none), False otherwise.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            attr_value = getattr(self.listener, attr_name)
            if attr_value in possible_values or (not_none and attr_value is not None):
                return True
            time.sleep(SHORT_SLEEP_1)

        return False

    def armCar(self):
        """
        Arms the car for operation.

        Returns:
            bool: True if the car is successfully armed within the timeout, False otherwise.
        """
        self.master.arducopter_arm()
        return self.timeout_check_listener("armed", 20, [True], False)

    def getHomePosition(self):
        """
        Retrieves the home position of the car.
        """
        self.printVerbose("Waiting for home position")

        # Request and wait for the home position message
        while True:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0,
                0, 0, 0, 0, 0, 0, 0, 0
            )

            self.home = self.master.recv_match(type='HOME_POSITION', blocking=False)

            if self.home is not None:
                break

            time.sleep(MEDIUM_SLEEP_1)


    
    def _do_action(self, action):
        """
        Performs an action by sending throttle and steering commands.

        Args:
            action: A tuple containing throttle and steering values.
        """

        throttle = ((action[0] - self.throttle_resolution) * self.throttle_magnitude) / self.throttle_resolution
        steering = ((action[1] - self.steering_resolution) * self.steering_magnitude) / self.steering_resolution

        self.printVerbose(f"Speed: {throttle}, Steering: {steering}")

        self._send_throttle_steering(throttle=throttle, steering=steering)
        
        self._compute_reward()

        if not self.override_mode:        
            self.override_mode = True

        time.sleep(self.len_timestep)

    def use_autopilot(self):
        """
        Stops the override mode to allow the vehicle to use autopilot.
        """

        if self.override_mode:
            try:
                self._send_command_to_server("override stop")
                self.override_mode = False
            except Exception as e:
                self.printVerbose(f"Error sending command: {e}")

            self.override_mode = False

    def _send_command_to_server(self, command):
        """
        Sends a command to the server.

        Args:
            command (str): The command to be sent to the server.
        """

        host = '127.0.0.1'
        port = 14551
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        
        try:
            s.connect((host, port))
            s.sendall(command.encode())
        except socket.timeout:
            self.printVerbose("Socket operation timed out.")
            self.done_flag = True
        except Exception as e:
            self.printVerbose(f"Error sending command: {e}")
        finally:
            s.close()

    def _send_throttle_steering(self, throttle, steering):
        """
        Sends throttle and steering commands to the server.

        Args:
            throttle (float): The throttle value to be sent.
            steering (float): The steering value to be sent.
        """

        command = f"override start {throttle} {steering}"
        try:
            self._send_command_to_server(command)
        except Exception as e:
            self.printVerbose(f"Error sending command: {e}")
    
    def update_car_position(self):
        """
        Updates the car's position and calculates distances to waypoints.
        """
        # Update past position and distances
        self.past_position = self.current_position
        self.past_distance_to_current_wp = self.current_distance_to_current_wp
        self.past_distance_to_path = self.current_distance_to_path

        # Attempt to get current state of the car
        try:
            carState = self.car.getCarState()
        except:
            self.full_reset_flag = True
            return None

        # Update current position
        carEstimatedPosition = carState.kinematics_estimated.position
        self.current_position = Point3D(carEstimatedPosition.x_val, carEstimatedPosition.y_val, carEstimatedPosition.z_val)

        # Calculate current waypoint position
        current_wp_index = min(len(self.waypoints) - 1, self.current_wp + 3)
        current_wp_x, current_wp_y = DistanceCalculator.lat_lon_to_meters(
            self.waypoints[current_wp_index][0], self.waypoints[current_wp_index][1],
            self.home.latitude / 1e7, self.home.longitude / 1e7
        )
        current_wp_z = -1 * self.waypoints[current_wp_index][2]
        self.current_waypoint_position = Point3D(current_wp_x, current_wp_y, current_wp_z)

        # Calculate past waypoint position
        past_wp_index = max(self.current_wp - 3, 0)
        past_wp_x, past_wp_y = DistanceCalculator.lat_lon_to_meters(
            self.waypoints[past_wp_index][0], self.waypoints[past_wp_index][1],
            self.home.latitude / 1e7, self.home.longitude / 1e7
        )
        past_wp_z = -1 * self.waypoints[past_wp_index][2]
        self.past_waypoint_position = Point3D(past_wp_x, past_wp_y, past_wp_z)

        # Update distances
        self.current_distance_to_current_wp = DistanceCalculator.distance_between_points(self.current_position, self.current_waypoint_position)
        self.current_distance_to_path = self.get_current_distance()

    def get_current_distance(self):
        """
        Calculates the car's current distance from the path.

        Returns:
            float: The minimum distance from the car to the path.
        """
        min_path_distance = 100

        for i in range(9):
            index_shift = 2 * i + 1

            # Get future waypoint position
            current_wp_index = min(len(self.waypoints) - 1, self.current_wp + index_shift)
            current_wp_x, current_wp_y = DistanceCalculator.lat_lon_to_meters(
                self.waypoints[current_wp_index][0], self.waypoints[current_wp_index][1],
                self.home.latitude / 1e7, self.home.longitude / 1e7
            )
            current_wp_z = -1 * self.waypoints[current_wp_index][2]
            future_wp = Point3D(current_wp_x, current_wp_y, current_wp_z)

            # Get past waypoint position
            past_wp_index = max(self.current_wp - index_shift, 0)
            past_wp_x, past_wp_y = DistanceCalculator.lat_lon_to_meters(
                self.waypoints[past_wp_index][0], self.waypoints[past_wp_index][1],
                self.home.latitude / 1e7, self.home.longitude / 1e7
            )
            past_wp_z = -1 * self.waypoints[past_wp_index][2]
            past_wp = Point3D(past_wp_x, past_wp_y, past_wp_z)

            # Calculate distance to path
            current_distance_to_path = min(
                min(DistanceCalculator.get_distance_to_path(self.current_position, past_wp, future_wp),
                    DistanceCalculator.distance_between_points(self.current_position, future_wp)),
                DistanceCalculator.distance_between_points(self.current_position, past_wp)
            )

            if current_distance_to_path < min_path_distance:
                min_path_distance = current_distance_to_path

        return min_path_distance

    def _compute_reward(self):
        """
        Computes the reward based on the car's position relative to the path and waypoints.

        Returns:
            float: The calculated reward.
        """
        self.update_car_position()

        # Calculate distance changes
        delta_to_path = self.past_distance_to_path - self.current_distance_to_path
        delta_to_waypoint = self.past_distance_to_current_wp - self.current_distance_to_current_wp

        # Update waypoint index if needed
        if self.current_wp != self.wp_listener.current_wp:
            self.current_wp = self.wp_listener.current_wp
            self.update_car_position()

        # Calculate reward for returning to the path
        return_to_path_reward = delta_to_path * 4
        return_to_path_reward = max(min(return_to_path_reward, 8), -8)

        # Calculate reward (penalty) for being far from the path
        path_proximity_reward = -2 if self.current_distance_to_path >= 2 else 0

        # Calculate final reward
        reward = return_to_path_reward + path_proximity_reward

        # Stalling detection
        if not self.override_mode:
            if abs(delta_to_waypoint) < 0.50:
                self.stalling_count += 1
            else:
                self.stalling_count = 0

        # Stalling condition
        if self.stalling_count >= 20:
            self.done_flag = True
            self.printVerbose("Stalling done flag")

        # Collision detection
        if self.car.simGetCollisionInfo().has_collided:
            reward = -100
            if self.current_attack and not self.saftey_policy_hit:
                self.saftey_policy_hit = True
                self.saftey_policy_collisions += 1
            self.done_flag = True
            self.printVerbose("Collision done flag")

        # Stay within bounds
        if self.current_distance_to_path > 40.0:
            self.done_flag = True
            self.printVerbose("Path done flag")

        # Check if final waypoint is reached
        if self.current_wp >= len(self.waypoints) - 1:
            self.done_flag = True
            self.printVerbose("Waypoint done flag")

        return reward

    def fetch_sensor_data(self):
        """
        Fetches and processes sensor data from the car.

        Returns:
            np.ndarray: Processed sensor data.
        """
        # Attempt to get current state of the car
        try:
            car_state = self.car.getCarState()
        except:
            self.full_reset_flag = True
            return np.zeros(SENSOR_SIZE)

        # Extracting kinematic values
        angular_velocity = np.array([car_state.kinematics_estimated.angular_velocity.x_val,
                                     car_state.kinematics_estimated.angular_velocity.y_val,
                                     car_state.kinematics_estimated.angular_velocity.z_val])
        
        angular_acceleration = np.array([car_state.kinematics_estimated.angular_acceleration.x_val,
                                     car_state.kinematics_estimated.angular_acceleration.y_val,
                                     car_state.kinematics_estimated.angular_acceleration.z_val])

        linear_acceleration = np.array([car_state.kinematics_estimated.linear_acceleration.x_val,
                                     car_state.kinematics_estimated.linear_acceleration.y_val,
                                     car_state.kinematics_estimated.linear_acceleration.z_val])
        
        linear_velocity = np.array([car_state.kinematics_estimated.linear_velocity.x_val,
                                     car_state.kinematics_estimated.linear_velocity.y_val,
                                     car_state.kinematics_estimated.linear_velocity.z_val])
        
        orientation = np.array([car_state.kinematics_estimated.orientation.w_val,
                                    car_state.kinematics_estimated.orientation.x_val,
                                     car_state.kinematics_estimated.orientation.y_val,
                                     car_state.kinematics_estimated.orientation.z_val])
        
        self.past_gps_position = self.current_gps_position
        
        carEstimatedPosition = car_state.kinematics_estimated.position
        self.current_gps_position = Point3D(carEstimatedPosition.x_val, carEstimatedPosition.y_val, carEstimatedPosition.z_val)

        gps_position = np.array([self.current_gps_position.x - self.past_gps_position.x,
                                 self.current_gps_position.y - self.past_gps_position.y,
                                 self.current_gps_position.z - self.past_gps_position.z])

        # Extract distance sensor data
        sensors = ["DistanceNorth", "DistanceEast", "DistanceSouth", "DistanceWest"]
        distance_sensor_vector = np.array([getattr(self.car.getDistanceSensorData(sensor), 'distance') for sensor in sensors])

        # Combine and reshape sensor data
        data_components = [angular_velocity, angular_acceleration, linear_acceleration, linear_velocity, distance_sensor_vector, orientation, gps_position]
        data = np.concatenate([component.reshape(1, -1) for component in data_components], axis=1).reshape(-1)

        return data

    def _get_obs(self):
        """
        Retrieves and formats the current observation from sensor data.

        Returns:
            np.ndarray: The current observation.
        """
        new_sensor_data = self.fetch_sensor_data()
        self.observation_buffer.append(new_sensor_data)

        # Format the observation buffer
        if len(self.observation_buffer) < self.buffer_size:
            padded_buffer = np.zeros((self.buffer_size, SENSOR_SIZE))
            padded_buffer[-len(self.observation_buffer):] = np.array(self.observation_buffer)
        else:
            padded_buffer = np.array(self.observation_buffer)

        observation = np.reshape(padded_buffer, (self.buffer_size, SENSOR_SIZE))
        return observation



    