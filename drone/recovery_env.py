"""
## Import Libraries
"""

import math
from math import atan2, asin, degrees
import socket
import subprocess
import threading
import time
from collections import deque
from pymavlink import mavutil
from dronekit import connect, Command, LocationGlobalRelative
from gym import spaces
from pyproj import Proj
import signal
import queue
import os
import gym
import airsim
import numpy as np

"""
## Constants
"""

SENSOR_SIZE = 21
MAV_MODE_AUTO = 4
SHORT_SLEEP_1 = 1
SHORT_SLEEP_2 = 2
MEDIUM_SLEEP_1 = 5
MEDIUM_SLEEP_2 = 10
LONG_SLEEP = 100
GYRO_SPOOFING_SCALING_FACTOR = 5


class RecoveryEnv(gym.Env):
    
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
        Initializes the environment with necessary parameters and settings, sets up connections with simulation tools, and configures internal state.
        
        Args:
            connections (dict): Contains connection settings like IPs and ports for AirSim and PX4.
            simulation_settings (dict): Contains settings like buffer size and timestep length.
            paths (dict): Contains file paths for simulation resources.
            attack_queue, defense_queue, reward_queue (queue.Queue): Queues for managing simulation events and rewards.
            docker (bool): Indicates if running in a Docker container.
            verbose (bool): Toggles verbose logging.
            evaluate (bool): Indicates evaluation mode.
            logdir (str): Directory path for logs.
        """

        # initialize environment
        super(RecoveryEnv, self).__init__()

        # set render mode
        self.metadata = {"render.modes": ["rgb_array"]}

        # unpack simulation settings
        self.buffer_size = simulation_settings["buffer_size"]
        self.len_timestep = simulation_settings["len_timestep"]
        self.movement_factor = simulation_settings["movement_factor"]
        self.movement_resolution = simulation_settings["movement_resolution"]
        self.detection_delay = simulation_settings["detection_delay"]
        self.postattack_delay = simulation_settings["postattack_delay"]
        self.attack_cooldown = simulation_settings["attack_cooldown"]

        # Queues
        self.defense_queue = defense_queue
        self.reward_queue = reward_queue
        self.attack_queue = attack_queue

        # Evaluation variables
        self.attack_count = 0
        self.saftey_policy_collisions = 0
        self.saftey_policy_full_recoveries = 0
        self.saftey_policy_partial_recoveries = 0
        self.saftey_policy_far_from_path = 0

        # unpack paths
        self.airsim_directory = paths["airsim_directory"]
        self.airsim_binary_name = paths["airsim_binary_name"]
        self.px4_directory = paths["px4_directory"]
        self.mission_file = paths["mission_file"]

        # Set observation and action space
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.buffer_size, SENSOR_SIZE), dtype=np.float32)
        self.action_space = spaces.MultiDiscrete([1 + 2 * self.movement_resolution, 1 + 2 * self.movement_resolution, 1 + 2 * self.movement_resolution])

        # Flags
        self.docker = docker
        self.verbose = verbose

        # Save cwd
        self.cwd = os.getcwd()
        self.evaluate = 1
        self.logdir = logdir
        self.recovered_time = 0

        # State variables
        self.current_distance_to_wp = -1
        self.past_distance_to_wp = -1

        self.x0 = 0
        self.y0 = 0
        self.z0 = 0
        
        self.past_gps = np.array([0.0, 0.0, 0.0])
        self.current_gps = np.array([0.0, 0.0, 0.0])
        self.current_wp = 0

        self.home = None
        self.full_reset_flag = False
        self.attack_reward = 0
        
        # Reward parameters
        self.path_threshold = 30
        self.wp_threshold = 5
        self.forward_progress_reward = 1
        self.backward_progress_reward = 1
        self.path_reward = 2
        self.new_wp_reward = 5
        self.reward_clipping_bound = 5
        self.collision_reward = -100

        # Unpack connections
        self.airsim_ip_address = connections["airsim_ip"]
        self.airsim_port = connections["airsim_port"]
        self.px4_override_ip = connections["px4_ip"]
        self.px4_override_port = connections["px4_port"]
        self.ekf2_port = connections["ekf2_port"]

        # Manually set flags
        self.override_mode = False # flag to see if RL has taken control
        self.arming_sequence = False # flag to see if in arming sequence
        self.home_position_set = False
        self.armed = False
        self.done_flag = True

        # evaluation
        self.recovery_threshold = 5.0
    
        # timing variables
        self.full_reset_time = LONG_SLEEP
        self.takeoff_timer = 20

        # reset flags
        self.full_reset_flag = False
        self.spoof_alt_offset = 0
        self.thread_flag = True
        self.stop_thread = False
        self.evaluation_result_recorded = False

        # Cleanup any open processes
        self.exit_airsim()
        self.exit_px4()
        time.sleep(SHORT_SLEEP_2) #nominal exit delay
    
        if self.docker:
            self.prepare_docker_build()

        # launch airsim
        self.launchAirsim()
        time.sleep(MEDIUM_SLEEP_2)

        # launch PX4
        self.launchPX4()
        time.sleep(MEDIUM_SLEEP_1)
        
       
        # airsim api
        self.drone = airsim.MultirotorClient(ip=self.airsim_ip_address)
        self.printVerbose("Connecting to the AirSim api...")

        time.sleep(MEDIUM_SLEEP_2)

        # mav link ground control
        self.vehicle_connection_string = "127.0.0.1:14550"
        self.vehicle = connect(self.vehicle_connection_string, wait_ready=True, timeout=60)

        time.sleep(MEDIUM_SLEEP_1)

        # connect to PX4 to override autopilot commands
        self.px4_override_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.px4_override_connection.connect((self.px4_override_ip, self.px4_override_port))

        # PX4 listener
        self.listener = self.message_listener(self)
        self.vehicle.add_message_listener('*', self.listener)

        # reset countdown thread
        self.countdown_thread = threading.Thread(target=self.reset_countdown, name="Countdown Thread")
        self.countdown_thread.start()

        self.done_flag = True

    def reset_countdown(self):
        """
        Runs a continuous monitoring thread to check for crashes in the simulation and trigger resets.

        Attributes:
            reset_counter (int): Counter to track the time elapsed during the arming sequence or since the last heartbeat.
            thread_flag (bool): Flag to control the running of this thread.
            full_reset_flag (bool): Flag that indicates if a full reset of the simulation is required.
            arming_sequence (bool): Indicates if the drone is currently in the arming sequence.
            last_heartbeat (float): Timestamp of the last heartbeat received from the drone.
        """
        # Reset the counter to 0 at the start.
        self.reset_counter = 0

        while self.thread_flag:
            # Check if a full reset is not already triggered.
            if not self.full_reset_flag:
                # Increment the counter during the arming sequence.
                if self.arming_sequence:
                    self.reset_counter += 1

                    # Trigger a full reset if the counter exceeds the time limit.
                    if self.reset_counter >= self.full_reset_time:
                        self.full_reset_flag = True
                        self.reset_counter = 0
                        self.printVerbose("TRIGGERED FULL RESET")
                else: 
                    # Reset the counter if not in the arming sequence.
                    self.reset_counter = 0

                # Capture the time of the last heartbeat.
                last_heartbeat = self.vehicle.last_heartbeat

                # Sleep for 1 second.
                time.sleep(SHORT_SLEEP_1)

                # Check if the heartbeat has not changed and trigger a full reset if so.
                if not (last_heartbeat != self.vehicle.last_heartbeat):
                    self.full_reset_flag = True
                    self.printVerbose("FULL RESET THROUGH HEARTBEAT")

        self.thread_flag = True

    def prepare_docker_build(self):
        
        """
        Prepares and launches the PX4 flight controller in a Docker environment.

        Attributes:
            px4_directory (str): Directory path of the PX4 flight controller software.
            px4_process (subprocess.Popen): Process object for the PX4 flight controller.
            cwd (str): Inital execution directory of the program.
        """        
        # Change to PX4 directory.
        os.chdir(self.px4_directory)

        # Remove any existing build directories.
        subprocess.check_call(["rm", "-rf", "build"])
                
        # Launch PX4 flight controller for test mode.
        self.px4_process = subprocess.Popen(
            ["make", "px4_sitl_default", "none_iris"], 
            stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        ) 

        self.printVerbose("Started PX4 Flight Controller")

        # Wait to ensure PX4 is operational.
        time.sleep(LONG_SLEEP)

        # Return to original working directory.
        os.chdir(self.cwd)

        # Exit the PX4 process.
        self.exit_px4()


    def launchAirsim(self):
        """
        Launches Microsoft AirSim simulation.

        Attributes:
            airsim_directory (str): Directory path of AirSim binary.
            airsim_process (subprocess.Popen): Process object for the AirSim binary.
            cwd (str): Current working directory of the program.
        """
        
        command = None

        if self.docker:
            command = ["./" + str(self.airsim_binary_name) + ".sh", "-RenderOffscreen", "-nosound", "-ResX=640", "-ResY=640", "-windowed", "-nullrhi"]
        else:
            command = ["gnome-terminal", "--", "./" + str(self.airsim_binary_name) + ".sh", "-ResX=640", "-ResY=640", "-windowed"]



        self.airsim_process = subprocess.Popen(command, cwd=self.airsim_directory)
        self.printVerbose("Started AirSim")
        os.chdir(self.cwd)

    
    def launchPX4(self):
        """
        Launches the PX4 flight controller for the simulation.

        Attributes:
            px4_directory (str): Directory path of the PX4 flight controller software.
            px4_process (subprocess.Popen): Process object for the PX4 flight controller.
            cwd (str): Current working directory of the program.
        """

        # Change to PX4 directory.
        os.chdir(self.px4_directory)

        # Launch PX4 flight controller
        self.px4_process = subprocess.Popen(
            ["make", "px4_sitl_default", "none_iris"], 
            stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )
        
        self.printVerbose("Started PX4 Flight Controller")

        # Return to original working directory.
        os.chdir(self.cwd)

    
                   
    
    @staticmethod
    def message_listener(instance):
        """
        Creates a MAVLink message listener for the simulation. Sets flags and updates the state of the simulation 
        based on the messages received from the simulation environment.

        Args:
            instance: An instance of the class to which this listener is attached.

        Returns:
            Function: A listener function that processes MAVLink messages.
        """

        def listener(self, name, message):
            # Process HOME_POSITION message
            if message.get_type() == "HOME_POSITION":
                instance.home_position_set = True

            # Process STATUSTEXT message for arming status
            elif message.get_type() == "STATUSTEXT":
                if message.severity == 6:
                    if "Armed" in message.text:
                        instance.printVerbose("Drone armed")
                        instance.armed = True
                    elif "Disarmed" in message.text:
                        instance.printVerbose("Drone disarmed")
                        instance.armed = False

            # Process HEARTBEAT message
            elif message.get_type() == "HEARTBEAT":
                instance.system_ready = True

            # Process SYS_STATUS message for sensor health
            elif message.get_type() == "SYS_STATUS":
                necessary_sensors = (
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    mavutil.mavlink.MAV_SYS_STATUS_AHRS
                )

                all_necessary_sensors_healthy = (message.onboard_control_sensors_health & necessary_sensors) == necessary_sensors
                instance.preflight_checks_passed = all_necessary_sensors_healthy

        return listener
                   
    
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

    def exit_airsim(self):
        """
        Terminates the currently running AirSim process.

        Attributes:
            airsim_binary_name (str): Name of the AirSim binary file.

        Exceptions:
            Handles any exceptions that may occur during the termination of AirSim processes.
        """
        # Notify about the attempt to kill AirSim process
        self.printVerbose("KILLING AirSim IF OPEN")

        try:
            # Get the process IDs for AirSim processes
            pid_bytes = subprocess.check_output(["pgrep", "-f", str(self.airsim_binary_name)])
            pid_string = pid_bytes.decode('utf-8')
            pids = pid_string.split()

            # Iterate over each PID and attempt to kill the corresponding process
            for pid in pids:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except Exception:
                    # Handle any exceptions silently
                    pass
        except Exception:
            # Handle any exceptions silently
            pass

    def exit_px4(self):
        """
        Terminates the PX4 process if it is running.

        Exceptions:
            Handles any exceptions that may occur during the termination of PX4 processes.
        """
        self.printVerbose("KILLING PX4 IF OPEN")

        try:
            # Get the process IDs for PX4 processes
            pid_bytes = subprocess.check_output(["pgrep", "-f", "px4"])
            pid_string = pid_bytes.decode('utf-8')
            pids = pid_string.split()

            # Iterate over each PID and attempt to kill the corresponding process
            for pid in pids:
                try:
                    self.printVerbose(f"Attempting to kill pid {pid}")
                    os.kill(int(pid), signal.SIGKILL)
                except Exception as e:
                    # Silently handle any exceptions
                    pass
        except Exception as e:
            # Silently handle any exceptions
            pass


    def restart_PX4_airsim(self):
        """
        Fully restarts the PX4 and AirSim environments. Also reinstates the drone client, vehicle connection,
        and the message listener.

        Exceptions:
            Handles exceptions silently and sets a full reset flag in case of any issues during the restart process.
        """
        self.printVerbose("--- FULL RESTARTING ---")

        # Flag to control threads
        self.thread_flag = False

        # Attempt to close vehicle connection
        try:
            self.vehicle.close()
        except:
            pass

        # Remove message listener
        try:
            self.vehicle.remove_message_listener('*', self.listener)
        except Exception as e:
            pass

        # Exit PX4 and AirSim with waits in between
        self.exit_px4()
        time.sleep(MEDIUM_SLEEP_2)

        self.exit_airsim()
        time.sleep(MEDIUM_SLEEP_2)

        # Launch AirSim and PX4 with waits in between
        self.launchAirsim()
        time.sleep(MEDIUM_SLEEP_2)

        self.launchPX4()
        time.sleep(MEDIUM_SLEEP_2)

        # Attempt to reinitialize connections
        try:
            # Reinitialize drone client
            self.drone = airsim.MultirotorClient(ip=self.airsim_ip_address)
            time.sleep(MEDIUM_SLEEP_2)

            # Connect to PX4
            self.vehicle = connect(self.vehicle_connection_string, wait_ready=True, timeout=60)
            time.sleep(MEDIUM_SLEEP_1)

            # Reestablish PX4 override connection
            self.px4_override_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.px4_override_connection.connect((self.px4_override_ip, self.px4_override_port))
            time.sleep(MEDIUM_SLEEP_1)

            # Re-add message listener
            self.printVerbose("RE ADDING MESSAGE LISTENER")
            self.listener = self.message_listener(self)
            self.vehicle.add_message_listener('*', self.listener)
            time.sleep(MEDIUM_SLEEP_1)

            # Restart countdown thread
            self.countdown_thread = threading.Thread(target=self.reset_countdown, name="Countdown Thread")
            self.countdown_thread.start()
            time.sleep(MEDIUM_SLEEP_1)

            # Indicate successful reset
            self.printVerbose("FULL RESET SUCCESSFUL")
            self.reset_counter = 0
            self.full_reset_flag = False
        except Exception as e:
            # Handle exceptions during restart
            self.printVerbose(f"Exception during restart: {e}")
            self.full_reset_flag = True
            

    def __del__(self):
        """
        Destructor for the environment, implemented to satisfy gym interface.
        """
        pass

   
    def wait_for_attack(self):
        """
        Waits for an attack command and initiates the corresponding attack.

        Returns:
            bool: True if an attack is successfully initiated, False otherwise.
        """
        # Reset current attack and transition to autopilot
        self.current_attack = None
        self.use_autopilot()


        # Wait for attack cooldown
        for i in range(self.attack_cooldown):
            self._compute_reward()
            time.sleep(SHORT_SLEEP_1)

        # Reset reward and attack active flag
        self.attack_reward = 0

        # Continuously feed observations to the attack agent
        while True:
            self._compute_reward()
            if self.done_flag:
                return False

            # Get and send observation
            obs = self._get_obs()
            self.defense_queue.put(obs)

            # Check for a new attack
            try:
                self.current_attack = self.attack_queue.get_nowait()
                break
            except queue.Empty:
                pass

            time.sleep(self.len_timestep)

        self._flush_attack_queue()

        # Start the attack if applicable
        if not self.done_flag and self.current_attack is not None:

            if (self.evaluation_result_recorded == False):
                self.saftey_policy_partial_recoveries += 1

            self.evaluation_result_recorded = False
            self.attack_count += 1

            # Initiate GPS or Gyro spoofing based on attack type
            if self.current_attack.type == "gps":
                self.spoof_gps(True, *self.current_attack.magnitude)
            elif self.current_attack.type == "gyro":
        
                self.spoof_gyro(True, *self.current_attack.magnitude)

            # Wait for detection delay
            for i in range(self.detection_delay):
                self._compute_reward()
                time.sleep(SHORT_SLEEP_1)

            self.update_drone_position()

            return True  # Attack successfully initiated

        return False
        

    def progress_attack(self):
        """
        Progresses the current attack and manage its duration.

        Returns:
            bool: True if the attack has completed, False if it's still ongoing.
        """
        # Check if attack duration has ended
        if self.current_attack.current_duration == 0:
            # Deactivate the attack
            if self.current_attack.type == "gps":
                self.spoof_gps(False, 0, 0, 0)
            elif self.current_attack.type == "gyro":
                self.spoof_gyro(False, 0, 0)

        # Track recovery time
        if self.get_distance_to_path() <= self.recovery_threshold:
            self.recovered_time += 1
        else:
            self.recovered_time = 0

        # Check if attack and recovery period has ended
        if self.current_attack.current_duration <= -self.postattack_delay:

            if (self.evaluation_result_recorded == False):
                if self.recovered_time >= (self.current_attack.total_duration / 4):
                    self.saftey_policy_full_recoveries += 1
                    self.evaluation_result_recorded = True
                

            self.current_attack = None
            return True
        else:
            # Decrement the attack duration
            self.current_attack.current_duration -= 1

        return False

    def step(self, action):
        """
        Performs a given predicted action, calculates the corresponding reward, then fetches and returns
        the new observation for the model.

        Args:
            action: The action to be performed in the simulation.

        Returns:
            tuple: A tuple containing the new observation, reward, done flag, and info dictionary.
            - obs: The new observation after performing the action.
            - reward: The computed reward as a result of the action.
            - done (bool): Flag indicating if the episode (attack instance) has ended.
            - info (dict): Additional information about the step.
        """
        info = {}
        done = False
        
        # Perform the given action
        self.override_autopilot(action)

        # Compute the reward for the action
        reward = self._compute_reward()

        self.printVerbose("Action: " + str(action) + ", Reward: " + str(reward))
        
        # Accumulate the attack reward
        self.attack_reward += reward

        # Check the status of the ongoing attack
        attack_done_flag = self.progress_attack()

        # Get a new observation
        obs = self._get_obs()

        # Check if the attack or simulation episode has ended
        if attack_done_flag or self.done_flag:
            done = True
            # Put accumulated attack reward in the queue
            self.reward_queue.put(self.attack_reward)

            # Switch back to autopilot
            self.use_autopilot()


        return obs, reward, done, info
    
    
    def stop_gps_spoofing(self):
        """
        Stops GPS spoofing by resetting the GPS spoof parameters.
        """
        try:
            self.spoof_gps(False, 0, 0, 0)
        except:
            pass

    def stop_gyro_spoofing(self):
        """
        Stops gyro spoofing by resetting the gyro spoof parameters.
        """
        try:
            self.spoof_gyro(False, 0, 0)
        except:
            pass


    def _flush_attack_queue(self):
        """
        Clears all items from the attack queue.
        """
        while not self.attack_queue.empty():
            _ = self.attack_queue.get()




    def reset(self):
        """
        Resets the environment for a new episode.

        Returns:
            obs: The initial observation after resetting the environment.
        """
        # Log evaluation metrics
        content = f"ATTACKS: {self.attack_count} | FULL RECOVERIES: {self.saftey_policy_full_recoveries} | PARTIAL RECOVERIES: {self.saftey_policy_partial_recoveries}  | COLLISIONS: {self.saftey_policy_collisions} | PATH VIOLATIONS: {self.saftey_policy_far_from_path}"
        self.update_evaluation_file(content)
        if (self.evaluate):
            print(content)

        # Setup flight if the episode is done
        if self.done_flag:
            self._setup_flight()

        # Wait for a new attack to start
        while not self.wait_for_attack():
            self._setup_flight()

        # Return the initial observation after reset
        return self._get_obs()
    

    def resetWorkingVars(self):
        """
        Resets working variables to their initial states.
        """

        self.past_gps = np.array([0.0, 0.0, 0.0])  # Reset past GPS coordinates
        self.current_gps = np.array([0.0, 0.0, 0.0])  # Reset current GPS coordinates
        self.recovered_time = 0  # Reset recovered time
        self.current_attack = None  # Clear current attack
        self.override_mode = False  # Reset override mode flag
        self.spoof_alt_offset = 0  # Reset GPS spoof altitude offset
        self.home_position_set = False  # Flag indicating if home position is set
        self.current_wp = 0  # Reset to first waypoint
        self.system_ready = False  # Flag indicating if the system is ready
        self.preflight_checks_passed = False  # Flag for preflight checks
        self.done_flag = False  # Flag indicating if the simulation is done
        self.observation_buffer = deque(maxlen=self.buffer_size)  # Initialize observation buffer
        self.min_distance = -1  # Reset minimum distance to waypoint
        self.past_min_dist = -1  # Reset past minimum distance to waypoint
        self.waypoint_num = 0
        
        

    
    def stop_override(self):
        """
        Stops overriding the drone's controls if it's currently active.
        """

        if self.override_mode:
            self.override_mode = False  # Disable override mode
            self.px4_tcp_override_command("stop\n")  # Send stop command to PX4


    def PX4forceDisarm(self):
        """
        Forces the PX4 to disarm the drone.
        """
        
        self.printVerbose("FORCING DISARM")

        try:
            # Send command to forcibly disarm the drone
            self.vehicle._master.mav.command_long_send(
                0, 0,  # target system and component
                400,   # command id for MAV_CMD_COMPONENT_ARM_DISARM
                0,     # confirmation
                0,     # disarm
                21196, # magic number for force disarm
                0, 0, 0, 0, 0  # unused parameters
            )
        except:
            self.full_reset_flag = True

        # Check if the drone has disarmed within the timeout period
        if not self.wait_for_condition(lambda: not self.armed, timeout=2.0):
            self.printVerbose("Timeout waiting for disarm to complete!")

    def PX4Reset(self):
        """
        Reboots the PX4 flight controller.
        """

        self.printVerbose("Rebooting PX4")

        # Define the target system and component for the command
        target_system = self.vehicle._master.target_system
        target_component = self.vehicle._master.target_component

        try:
            # Send command to reboot the PX4 flight controller
            self.vehicle._master.mav.command_long_send(
                target_system,  # Target system
                target_component,  # Target component
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # Command ID for reboot/shutdown
                0,  # Confirmation
                1,  # Reboot autopilot
                1,  # Reboot onboard computer
                1,  # Reboot component
                0, 0, 0, 0  # Unused parameters
            )
        except:
            self.full_reset_flag = True

        # Wait for the system to be ready post-reboot or until timeout
        if not self.wait_for_condition(lambda: self.system_ready, 20.0):
            self.printVerbose("Timeout reached for reboot.")
        
      
    def PX4arm(self):
        """
        Arms the drone using a MAVLink command.
        """

        try:
            # Encode and send a MAVLink command to arm the drone
            arm_command = self.vehicle._master.mav.command_long_encode(
                0, 0,  # Target system and component
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command for arm/disarm
                0,     # Confirmation
                1,     # Param 1, 1 to indicate arm
                0, 0, 0, 0, 0, 0  # Params 2-7 (not used)
            )

            # Send the arm command
            self.vehicle._master.mav.send(arm_command)
        except:
            self.full_reset_flag = True

    def resetEKF2(self):
        """
        Sends commands to reset the EKF2 (Extended Kalman Filter 2) in PX4.
        """
        
        self.printVerbose("EKF2 RESET")
        # Stop the EKF2 system
        self.PX4consoleCommand("ekf2 stop")
        time.sleep(SHORT_SLEEP_1)  # Short pause after stopping EKF2

        # Restart the EKF2 system
        self.PX4consoleCommand("ekf2 start")

        # Reset the flag indicating preflight checks are passed
        self.preflight_checks_passed = False

        # Wait for the EKF2 system to reset or timeout
        if not self.wait_for_condition(lambda: self.preflight_checks_passed, timeout=20.0):
            self.printVerbose("Timeout waiting for EKF2 to reset!")
            self.full_reset_flag = True  # Set flag indicating a need for full reset
            

    def resetAirsim(self):
        """
        Resets the AirSim simulation environment.
        """

        self.printVerbose("AirSim RESET")
        try:
            # Attempt to reset the AirSim environment
            self.drone.reset()
        except:
            # Set the flag for a full reset if AirSim reset fails
            self.full_reset_flag = True

        time.sleep(SHORT_SLEEP_2)  # Short pause after resetting AirSim


    def wait_for_condition(self, check_function, timeout):
        """
        Waits for a specified condition to be met within a given timeout period.

        Args:
            check_function (function): A function that returns a boolean value, indicating whether the condition is met.
            timeout (float): The maximum time in seconds to wait for the condition to be met.

        Returns:
            bool: True if the condition is met within the timeout, False otherwise.
        """

        start_time = time.time()  # Record the start time
        while True:
            if check_function():
                return True  # Condition is met
            elif time.time() - start_time > timeout:
                return False  # Timeout reached
            time.sleep(0.1)  # Short pause to avoid busy waiting

    def resetDrone(self):
        """
        Performs a reset of the drone's systems.
        """
        self.PX4forceDisarm()  # Force disarm the drone
        self.resetAirsim()     # Reset the AirSim simulation environment
        self.PX4Reset()        # Reboot the PX4 flight controller
        self.resetEKF2()       # Reset the EKF2 system on PX4

    def PX4missionStart(self):
        """
        Sends the mission start command on the PX4 flight controller.
        """

        try:
            self.vehicle._master.mav.command_long_send(
                self.vehicle._master.target_system,         # Target system
                self.vehicle._master.target_component,      # Target component
                mavutil.mavlink.MAV_CMD_MISSION_START,      # Command to start mission
                0,                                          # Confirmation
                0, 0, 0, 0,                                 # Reserved parameters, set to 0
                0,                                          # First item in the mission to execute
                0,                                          # Last item in the mission to execute
                0                                           # Reserved, set to 0
            )
        except:
            self.full_reset_flag = True

    def setupAutopilot(self):
        """
        Sets up the autopilot with a predefined mission.
        """

        worker_thread = threading.Thread(target=self.setupAutopilotWorker)
        worker_thread.start()
        worker_thread.join(timeout=10)  # 10-second timeout
        
        if worker_thread.is_alive():
            # Handle timeout, e.g., by logging, cleaning up, or taking corrective action
            print("Timeout occurred during autopilot setup.")

    def setupAutopilotWorker(self):
        """
        Worker function to be executed in a separate thread.
        This contains the original operations from setupAutopilot.
        """
        
        try:
            self.waypoints = []
            cmds = self.vehicle.commands
            cmds.clear()
            
            # Load new commands
            filename = self.mission_file
            new_commands = self.read_mission_file(filename, self.home)
            for cmd in new_commands:
                cmds.add(cmd)

            # Upload the mission and start it
            self.vehicle.commands.upload()
            self.printVerbose("Mission uploaded.")
            self.PX4missionStart()
            self.PX4setMode(MAV_MODE_AUTO)
        except Exception as e:
            print("Error occurred in Autopilot setup.")

    def override_autopilot(self, throttles):
        """
        Overrides the autopilot to manually control the drone's movements.

        Args:
            throttles (tuple): A tuple containing throttle values for forward/backward, left/right, and up/down movements.
        """
        scaling_factor = self.movement_factor
        # Unpack throttle inputs
        forward_backward, left_right, up_down = throttles

        

        # Map throttle inputs to local velocities
        local_vx = forward_backward - (self.movement_resolution)
        local_vy = left_right - (self.movement_resolution)
        vz = up_down - (self.movement_resolution)

        # Get the drone's current yaw angle
        imu_data = self.drone.getImuData()
        quaternion = imu_data.orientation
        euler = airsim.to_eularian_angles(quaternion)
        yaw = euler[2]  # Yaw angle

        # Calculate velocities in the Earth frame
        vx = local_vx * np.cos(yaw) - local_vy * np.sin(yaw)
        vy = local_vx * np.sin(yaw) + local_vy * np.cos(yaw)

        # Apply scaling factor
        vx = str(float(vx) * scaling_factor)
        vy = str(float(vy) * scaling_factor)
        vz = str(float(vz) * scaling_factor)

        # Construct and send command
        command = f"start {vx} {vy} {vz}\n"
        #print(command)
        self.px4_tcp_override_command(command)
        
        time.sleep(self.len_timestep)  # Pause for the specified timestep

        # Enable override mode if not already enabled
        if not self.override_mode:
            self.override_mode = True



    def use_autopilot(self):
        """
        Switches control back to the autopilot from manual override.
        """

        if self.override_mode:  # Check if override mode is active
            self.px4_tcp_override_command("stop\n")  # Send command to stop manual override
            self.override_mode = False  # Disable override mode

        time.sleep(self.len_timestep)  # Pause for the specified timestep

    def px4_tcp_override_command(self, command):
        """
        Sends a command to the PX4 flight controller over TCP.
        """

        try:
            command = command.encode('ascii')  # Encode the command to ASCII
            self.px4_override_connection.sendall(command)  # Send the command
        except:
            self.full_reset_flag = True  # Set full reset flag in case of an exception

    def spoof_gps(self, spoof, x, y, alt):
        """
        Initiates or stops GPS spoofing.

        Args:
            spoof (bool): True to start spoofing, False to stop.
            x (float): The spoofed x-coordinate.
            y (float): The spoofed y-coordinate.
            alt (float): The spoofed altitude.
        """
        command = None

        if spoof:
            self.spoof_alt_offset = alt
            self.gps_spoofing_flag = True
            self.printVerbose(f"GPS SPOOFING STARTED Z: {x} {y} {alt}")
            command = f"gps {x} {y} {alt}"
        else:
            self.printVerbose("GPS SPOOFING STOPPED")
            command = "stop gps"

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_address = ('localhost', self.ekf2_port)
            sock.connect(server_address)
            sock.sendall(command.encode())
        except:
            pass  # Silently handle exceptions
        finally:
            sock.close()  # Close the socket


    def spoof_gyro(self, spoof, axis, magnitude):
        """
        Initiates or stops gyro spoofing.

        Args:
            spoof (bool): True to start spoofing, False to stop.
            roll (float): The spoofed roll value.
            pitch (float): The spoofed pitch value.
            yaw (float): The spoofed yaw value.
        """

        # Scale the gyro spoofing values
        roll = 0
        pitch = 0
        yaw = 0


        #print(axis)
        if (axis == 0):
            roll = GYRO_SPOOFING_SCALING_FACTOR * magnitude
        elif (axis == 1):
            pitch = GYRO_SPOOFING_SCALING_FACTOR * magnitude
        elif (axis == 2):
            yaw = GYRO_SPOOFING_SCALING_FACTOR * magnitude


        if spoof:
            self.printVerbose(f"GYRO SPOOFING STARTED Z: {roll} {pitch} {yaw}")
            command = f"gyro {roll} {pitch} {yaw}"
        else:
            self.printVerbose("GYRO SPOOFING STOPPED")
            command = "stop gyro"

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_address = ('localhost', self.ekf2_port)
            sock.connect(server_address)
            sock.sendall(command.encode())
            sock.close()
        except:
            self.full_reset_flag = True  # Set full reset flag in case of an exception

    def _setup_flight(self):
        """
        Sets up the flight by preparing the drone and the simulation environment.
        """

        try:
            self.stop_gps_spoofing()
            self.stop_gyro_spoofing()
            self.stop_override()
        except:
            pass  # Silently handle exceptions

        self.arming_sequence = True  # Indicate the start of the arming sequence

        # Reboot and set up the vehicle until it's flight-ready
        while True:
            self.resetWorkingVars()  # Reset working variables
            self.resetDrone()  # Reset the drone position, PX4, and EKF2

            if self.full_reset_flag:
                self.restart_PX4_airsim()
                continue

            time.sleep(SHORT_SLEEP_2)

            # Wait for the home position to be set
            while not self.home_position_set and not self.full_reset_flag:
                self.printVerbose("Waiting for home position...")
                time.sleep(0.1)

            self.home = self.vehicle.location.global_relative_frame
            self.printVerbose(str(self.home))

            self.setupAutopilot()  # Load and start the mission
            self.PX4arm()  # Arm the vehicle

            time.sleep(MEDIUM_SLEEP_1)

            if not self.wait_for_condition(lambda: self.armed, 5.0):
                self.printVerbose("Timeout reached for arm check.")
                self.full_reset_flag = True
                continue

            # Confirm arm and check for takeoff
            if self.armed and not self.full_reset_flag:
                elapsed_time = 0
                takeoff = True
                while abs(self.drone.getMultirotorState().kinematics_estimated.position.z_val) < self.home.alt + 0.1:
                    elapsed_time += 1
                    time.sleep(1)
                    if elapsed_time >= self.takeoff_timer:
                        takeoff = False
                        break

                if takeoff and not self.full_reset_flag:
                    self.drone.simGetCollisionInfo().has_collided  # Clear initial collision
                    break

        self.printVerbose("ESCAPED ARMING SEQUENCE")
        self.arming_sequence = False
        self.vehicle.commands.next = 0


    
    def PX4consoleCommandWorker(self, command):
        try:
            command = command + "\n"
            self.px4_process.stdin.write(command)
            self.px4_process.stdin.flush()

            # Read and handle the response
            response = self.px4_process.stdout.readline()
            while response:
                response = self.px4_process.stdout.readline()
                if command in response or self.full_reset_flag:
                    break

            self.px4_process.stdout.flush()
        except Exception as e:
            self.full_reset_flag = True

    def PX4consoleCommand(self, command):
        # Create a thread for the worker function
        worker_thread = threading.Thread(target=self.PX4consoleCommandWorker, args=(command,))

        # Start the worker thread
        worker_thread.start()

        # Wait for the worker to complete or timeout
        worker_thread.join(timeout=5)

        # If the thread is still alive after the timeout, handle the timeout scenario
        if worker_thread.is_alive():
            print("Timeout occurred while sending PX4 console command")
            self.full_reset_flag = True
      

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Converts Euler angles to a quaternion representation.

        Args:
            roll (float): Roll angle in radians.
            pitch (float): Pitch angle in radians.
            yaw (float): Yaw angle in radians.

        Returns:
            np.array: The quaternion representation [qw, qx, qy, qz] of the given Euler angles.
        """

        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return np.array([qw, qx, qy, qz])

    def quaternion_multiply(self, quaternion1, quaternion0):
        """
        Multiplies two quaternions.

        Args:
            quaternion1 (np.array): The first quaternion [qw, qx, qy, qz].
            quaternion0 (np.array): The second quaternion [qw, qx, qy, qz].

        Returns:
            np.array: The product of the quaternion multiplication.
        """

        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1

        return np.array([
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
        ])


    def fetch_sensor_data(self):
        """
        Fetches and processes various sensor data from the drone.

        Returns:
            processed_observation: The processed observation data ready for use in the simulation environment.
        """
        
        # Retrieve multirotor state
        multirotor_state = self.drone.getMultirotorState()

        # Retrieve and process the drone's linear velocity
        quad_vel = multirotor_state.kinematics_estimated.linear_velocity
        quad_vel = np.array([quad_vel.x_val, quad_vel.y_val, quad_vel.z_val])

        # Retrieve and process IMU data
        imu_data = self.drone.getImuData()
        angular_velocity = np.array([imu_data.angular_velocity.x_val, imu_data.angular_velocity.y_val, imu_data.angular_velocity.z_val])
        linear_acceleration = np.array([imu_data.linear_acceleration.x_val, imu_data.linear_acceleration.y_val, imu_data.linear_acceleration.z_val])
        orientation = np.array([imu_data.orientation.w_val, imu_data.orientation.x_val, imu_data.orientation.y_val, imu_data.orientation.z_val])

        gyro_state = self.quaternion_to_euler_degrees(orientation)

        if (self.current_attack != None):

            if (self.current_attack.magnitude[0] == 0):
                gyro_state[0] = gyro_state[0] + self.current_attack.magnitude[1] * GYRO_SPOOFING_SCALING_FACTOR
            elif (self.current_attack.magnitude[0] == 1):
                gyro_state[1] = gyro_state[1] + self.current_attack.magnitude[1] * GYRO_SPOOFING_SCALING_FACTOR
            elif (self.current_attack.magnitude[0] == 2):
                gyro_state[2] = gyro_state[2] + self.current_attack.magnitude[1] * GYRO_SPOOFING_SCALING_FACTOR

        # Retrieve distance sensor data
        distance_sensors = {
            "North": self.drone.getDistanceSensorData("DistanceNorth"),
            "East": self.drone.getDistanceSensorData("DistanceEast"),
            "South": self.drone.getDistanceSensorData("DistanceSouth"),
            "West": self.drone.getDistanceSensorData("DistanceWest"),
            "Up": self.drone.getDistanceSensorData("DistanceUp"),
            "Down": self.drone.getDistanceSensorData("DistanceDown")
        }
        distance_sensor_data = np.array([sensor.distance for sensor in distance_sensors.values()])

        # Retrieve and calculate GPS state
        gps_position = multirotor_state.kinematics_estimated.position
        self.past_gps = self.current_gps
        self.current_gps = np.array([gps_position.x_val, gps_position.y_val, gps_position.z_val])
        gps_state = self.current_gps - self.past_gps

        # Combine all sensor data into a single observation
        observation = [quad_vel, angular_velocity, linear_acceleration, gyro_state, gps_state, distance_sensor_data]

        return self.process_state(observation)
    

    def _get_obs(self):
        """
        Retrieves the current observation from the simulation environment.

        Returns:
            observation: A reshaped array of sensor data representing the current observation.
        """

        # Fetch new sensor data
        new_sensor_data = self.fetch_sensor_data()
        sensor_data = np.array(new_sensor_data)

        # Append the new sensor data to the observation buffer
        self.observation_buffer.append(sensor_data)

        # If the buffer isn't full, pad it with zeros
        if len(self.observation_buffer) < self.buffer_size:
            padded_buffer = np.zeros((self.buffer_size, SENSOR_SIZE))
            padded_buffer[-len(self.observation_buffer):] = np.array(self.observation_buffer)
        else:
            # Use the full buffer if it's already filled
            padded_buffer = np.array(self.observation_buffer)
        
        # Reshape the observation to ensure it's a 3D array
        observation = np.reshape(padded_buffer, (self.buffer_size, SENSOR_SIZE))

        return observation

    def quaternion_to_euler_degrees(self, q):
        
        w, x, y, z = q
    
        # Roll (x-axis rotation)
        roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        roll_degrees = degrees(roll)
        
        # Pitch (y-axis rotation)
        pitch_sin = 2.0 * (w * y - z * x)
        pitch_sin = min(1.0, max(-1.0, pitch_sin))
        pitch = asin(pitch_sin)
        pitch_degrees = degrees(pitch)
        
        # Yaw (z-axis rotation)
        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw_degrees = degrees(yaw)
        
        return np.array([roll_degrees, pitch_degrees, yaw_degrees])

    def process_state(self, state):
        """
        Processes the raw state data into a format suitable for StableBaselines.

        Args:
            state (tuple): A tuple containing various components of the drone's state.

        Returns:
            np.array: A flattened array representing the processed state.
        """

        # Unpack the state components
        vel, angular_velocity, linear_acceleration, gyro_state, gps, distance_sensors = state

        # Reshape each component for concatenation
        vel = vel.reshape(1, -1)
        angular_velocity = angular_velocity.reshape(1, -1)
        linear_acceleration = linear_acceleration.reshape(1, -1)
        gyro_state = gyro_state.reshape(1, -1)
        gps = gps.reshape(1, -1)
        distance_sensors = distance_sensors.reshape(1, -1)

        # Concatenate all components into a single array
        data = np.concatenate((vel, angular_velocity, linear_acceleration, gyro_state, gps, distance_sensors), axis=1)

        # Flatten the array to create a single-dimensional representation of the state
        flattened_state = data.reshape(-1)

        return flattened_state


    def shortest_distance(self, p0, p1, p2):
        """
        Calculates the shortest distance from a point to a line segment.

        Args:
            p0 (list or np.array): The point from which the distance is to be measured.
            p1 (list or np.array): One endpoint of the line segment.
            p2 (list or np.array): The other endpoint of the line segment.

        Returns:
            float: The shortest distance from point p0 to the line segment between p1 and p2.
        """

        # Convert points to numpy arrays for vector operations
        v = np.array(p2) - np.array(p1)
        w = np.array(p0) - np.array(p1)

        # Compute squared length of the segment
        length_squared = np.dot(v, v)

        # Handle case where p1 and p2 are the same (segment length is zero)
        if length_squared == 0:
            return np.linalg.norm(w)

        # Find the projection of point p0 onto the line
        dot_product = np.dot(w, v)
        t = dot_product / length_squared
        t = max(0, min(t, 1))  # Clamp t to the range [0, 1]

        # Compute the closest point on the segment to p0
        p_closest = np.array(p1) + t * v

        # Return the distance from p0 to the closest point on the segment
        return np.linalg.norm(np.array(p0) - p_closest)

    def distance_between_points(self, p0, p1):
        """
        Calculates the Euclidean distance between two points in 3D space.

        Args:
            p0 (list or np.array): The first point [x1, y1, z1].
            p1 (list or np.array): The second point [x2, y2, z2].

        Returns:
            float: The distance between points p0 and p1.
        """
        x1, y1, z1 = p0
        x2, y2, z2 = p1
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def get_distance_to_path(self):
        """
        Calculates the minimum distance of the drone to its flight path.

        Returns:
            float: The minimum distance of the drone to its flight path.
        """
        past_wp = max(self.current_wp - 1, 0)
        p0 = [self.x0, self.y0, self.z0]

        # Coordinates for the past waypoint
        x1, y1 = self.lat_lon_to_meters(self.waypoints[past_wp][0], self.waypoints[past_wp][1], self.home.lat, self.home.lon)
        z1 = -1 * self.waypoints[past_wp][2] if self.current_wp > 0 else self.home.alt
        p1 = [x1, y1, z1]

        # Coordinates for the next waypoint
        x2, y2 = self.lat_lon_to_meters(self.waypoints[self.current_wp][0], self.waypoints[self.current_wp][1], self.home.lat, self.home.lon)
        z2 = -1 * self.waypoints[self.current_wp][2]
        p2 = [x2, y2, z2]

        # Calculate the minimum distance to the path
        return min(
            self.get_distance_to_past_wp(),
            min(self.shortest_distance(p0, p1, p2), self.get_distance_to_next_wp())
        )

    def update_evaluation_file(self, content):
        """
        Updates an evaluation file with the given content.

        Args:
            content (str): The content to append to the evaluation file.
        """

        if self.evaluate > 0:
            with open(os.path.join(self.logdir, 'evaluation.txt'), 'a') as file:
                file.write(content + '\n')

    def update_drone_position(self):
        """
        Updates the drone's current position based on its state and any ongoing attacks.

        This function retrieves the drone's position from the simulation and adjusts it if there's an ongoing GPS spoofing attack.
        """

        self.position = self.drone.getMultirotorState().kinematics_estimated.position
        self.x0 = self.position.x_val
        self.y0 = self.position.y_val
        self.z0 = self.position.z_val


        # Adjust position if there's a GPS spoofing attack
        if self.current_attack and self.current_attack.type == "gps" and self.current_attack.current_duration >= 0:
            spoof_offset = self.spoof_alt_offset
            self.z0 = self.z0 - spoof_offset if spoof_offset > 0 else max(self.z0 - spoof_offset, self.z0)

    def get_distance_to_next_wp(self):
        """
        Calculates the distance from the drone to the next waypoint.

        Returns:
            float: The distance from the drone's current position to the next waypoint.
        """

        p0 = [self.x0, self.y0, self.z0]
        x2, y2 = self.lat_lon_to_meters(self.waypoints[self.current_wp][0], self.waypoints[self.current_wp][1], self.home.lat, self.home.lon)
        z2 = -1 * self.waypoints[self.current_wp][2]
        p2 = [x2, y2, z2]
        return self.distance_between_points(p0, p2)

    def get_distance_to_past_wp(self):
        """
        Calculates the distance from the drone to the past waypoint.

        Returns:
            float: The distance from the drone's current position to the past waypoint.
        """

        past_wp = max(self.current_wp - 1, 0)
        p0 = [self.x0, self.y0, self.z0]
        x1, y1 = self.lat_lon_to_meters(self.waypoints[past_wp][0], self.waypoints[past_wp][1], self.home.lat, self.home.lon)
        z1 = -1 * self.waypoints[past_wp][2] if self.current_wp > 0 else self.home.alt
        p1 = [x1, y1, z1]
        return self.distance_between_points(p0, p1)
    
        
    def _compute_reward(self):

        # initialize reward and done variables
        done = False
        done_reason = ""

        # update position and distances
        self.update_drone_position()

        new_waypoint = False
        
        if self.get_distance_to_next_wp() < self.wp_threshold:

            if self.current_wp >= len(self.waypoints) - 1:
                self.done_flag = True
                return self.new_wp_reward
            else:
                new_waypoint = True
                self.current_distance_to_wp = -1
                self.past_distance_to_wp = -1
                self.current_wp += 1
                self.waypoint_num += 1
                self.vehicle.commands.next = self.waypoint_num

        
                

        # update current distances to waypoint, path, etc.
        if self.current_distance_to_wp == -1 or self.past_distance_to_wp == -1:
            self.current_distance_to_wp = self.get_distance_to_next_wp()
            self.past_distance_to_wp = self.get_distance_to_next_wp()

        self.past_distance_to_wp = self.current_distance_to_wp
        self.current_distance_to_wp = self.get_distance_to_next_wp()
        
        self.distance_to_path = self.get_distance_to_path()
        self.distance_to_past_wp = self.get_distance_to_past_wp()

        if self.past_min_dist == -1:
            self.past_min_dist = min(self.distance_to_path, self.current_distance_to_wp, self.distance_to_past_wp)
        else:
            self.past_min_dist = self.min_distance

        self.min_distance = min(self.distance_to_path, self.current_distance_to_wp, self.distance_to_past_wp)
    
        # calculate distance metrics used in reward calculation
        delta_to_path = self.past_min_dist - self.min_distance
        delta_to_waypoint = self.past_distance_to_wp - self.current_distance_to_wp

        # calculate reward for returning to the path
        path_return_reward = 0
        if delta_to_path >= 0.2:
            path_return_reward = delta_to_path * self.path_reward
        elif delta_to_path <= -0.2:
            path_return_reward = delta_to_path * self.path_reward

        path_return_reward = max(min(path_return_reward, self.reward_clipping_bound), -1 * self.reward_clipping_bound)


        # calculate reward for progressing to next waypoint
        waypoint_reward = 0
    
        if delta_to_waypoint >= 0:
            waypoint_reward = delta_to_waypoint * self.forward_progress_reward
        else:
            waypoint_reward = delta_to_waypoint * self.backward_progress_reward

        waypoint_reward = max(min(waypoint_reward, self.reward_clipping_bound), -1 * self.reward_clipping_bound)    

        # calculate final reward
        reward = path_return_reward + waypoint_reward

        if (new_waypoint == True):
            reward = max(reward, self.new_wp_reward)
          
        # collision
        if self.drone.simGetCollisionInfo().has_collided:
            reward = self.collision_reward
            if self.evaluation_result_recorded == False:
                self.saftey_policy_collisions += 1
                self.evaluation_result_recorded = True

            self.done_flag = True
        

        # stay within bounds
        if self.min_distance > self.path_threshold:
            self.done_flag = True

            if self.evaluation_result_recorded == False:
                self.saftey_policy_far_from_path += 1
                self.evaluation_result_recorded = True

        
        # px4 or other software errors results in a full reset.
        if self.full_reset_flag == True:
            self.done_flag = True
        
    
        return reward

    def lat_lon_to_meters(self, latitude, longitude, reference_latitude, reference_longitude):
        """
        Converts latitude and longitude to meters using a Universal Transverse Mercator (UTM) projection.

        Args:
            latitude (float): Latitude to be converted.
            longitude (float): Longitude to be converted.
            reference_latitude (float): Reference latitude for the UTM projection.
            reference_longitude (float): Reference longitude for the UTM projection.

        Returns:
            tuple: The converted coordinates (x, y) in meters.
        """

        p = Proj(proj='utm', zone=10, ellps='WGS84')
        reference_easting, reference_northing = p(reference_longitude, reference_latitude)
        easting, northing = p(longitude, latitude)
        return northing - reference_northing, easting - reference_easting

    def PX4setMode(self, mavMode):
        """
        Sets the PX4 flight controller to a specified mode.

        Args:
            mavMode (int): The MAVLink mode to set.
        """

        try:
            self.vehicle._master.mav.command_long_send(
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0, mavMode, 0, 0, 0, 0, 0, 0
            )
        except:
            self.full_reset_flag = True

    def get_bearing(self, pointA, pointB):
        """
        Calculates the initial bearing from one point to another.

        Args:
            pointA (LocationGlobalRelative): The starting point.
            pointB (LocationGlobalRelative): The destination point.

        Returns:
            float: The initial bearing in degrees.
        """

        lat1 = math.radians(pointA.lat)
        lat2 = math.radians(pointB.lat)
        diffLong = math.radians(pointB.lon - pointA.lon)
        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(diffLong)
        initial_bearing = math.atan2(x, y)
        compass_bearing = (math.degrees(initial_bearing) + 360) % 360
        return compass_bearing
    

    def read_mission_file(self, filename, home_position):
        """
        Reads a mission file and processes the waypoints.

        Args:
            filename (str): The path to the mission file.
            home_position (LocationGlobalRelative): The home position for adjusting waypoint coordinates.

        Returns:
            list: A list of Command objects representing the mission waypoints.
        """

        mission_list = []
        first_waypoint = None
        previous_waypoint = None
        takeoff = False

        with open(filename) as f:
            for i, line in enumerate(f):
                if i == 0:
                    # Check for correct file format
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    line_array = line.split('\t')

                    # Identify the takeoff command to start processing waypoints
                    if not takeoff:
                        if int(line_array[3]) == 22:  # MAV_CMD_NAV_TAKEOFF
                            takeoff = True

                    if takeoff:
                        # Parse the waypoint data
                        ln_frame = int(line_array[2])
                        ln_command = int(line_array[3])
                        ln_currentwp = int(line_array[1])
                        ln_autocontinue = int(line_array[11])
                        ln_param1 = float(line_array[4])
                        ln_param2 = float(line_array[5])
                        ln_param3 = float(line_array[6])
                        ln_param4 = float(line_array[7])
                        ln_param5 = float(line_array[8])
                        ln_param6 = float(line_array[9])
                        ln_param7 = float(line_array[10])

                        # Adjust waypoint coordinates based on the first waypoint and home position
                        if first_waypoint is None:
                            first_waypoint = LocationGlobalRelative(ln_param5, ln_param6, ln_param7)
                        ln_param5 = ln_param5 - first_waypoint.lat + home_position.lat
                        ln_param6 = ln_param6 - first_waypoint.lon + home_position.lon
                        ln_param7 = ln_param7

                        # Store the adjusted waypoint
                        self.waypoints.append([ln_param5, ln_param6, ln_param7])
                        current_waypoint = LocationGlobalRelative(ln_param5, ln_param6, ln_param7)

                        # Calculate and assign bearing for navigation if applicable
                        if previous_waypoint is not None and ln_command in [mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM]:
                            ln_param4 = self.get_bearing(previous_waypoint, current_waypoint)

                        # Create and add the command to the mission list
                        cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue,
                                    ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                        mission_list.append(cmd)
                        previous_waypoint = current_waypoint

        return mission_list