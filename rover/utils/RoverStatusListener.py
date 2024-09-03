import threading
import time
from pymavlink import mavutil, mavwp

HEART_BEAT_STABILITY = 2.0

EKF_HEALTHY_FLAG = 831

class RoverStatusListener(threading.Thread):

    def __init__(self, master):
        super(RoverStatusListener, self).__init__()
        self.master = master
        self.keep_running = True 

        # fields
        self.resetFields()
        

    def resetFields(self):
        self.current_mode = None
        self.armed = False
        self.system_status = 0
        self.mission_request = None
        self.heartbeat_stable = True
        self.last_heartbeat_time = None
        self.gps_status = 0
        self.ekf_healthy = False
        self.current_wp = 0


    def run(self):

        while self.keep_running:


            heartbeat_msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if heartbeat_msg is not None:

         
                if (not mavutil.mode_string_v10(heartbeat_msg) is None):
                    self.current_mode = mavutil.mode_string_v10(heartbeat_msg)
       
                if (not heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED is None):
                    if heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        self.armed = True
                    else:
                        self.armed = False

                if (not heartbeat_msg.system_status is None):
                    self.system_status = heartbeat_msg.system_status
                
                # Update last heartbeat time
                self.last_heartbeat_time = time.time()

            # Check for heartbeat stability
            if self.last_heartbeat_time:
                elapsed_time_since_last_heartbeat = time.time() - self.last_heartbeat_time
                # Let's assume that if we haven't received a heartbeat in more than 2 seconds, it's unstable.
                self.heartbeat_stable = elapsed_time_since_last_heartbeat <= HEART_BEAT_STABILITY
            else:
                self.heartbeat_stable = False

            gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=False)
            if gps_msg is not None:
                self.gps_status = gps_msg.fix_type
    

            ekf_status_msg = self.master.recv_match(type='EKF_STATUS_REPORT', blocking=False)
            if ekf_status_msg is not None:
                
                if ekf_status_msg.flags == EKF_HEALTHY_FLAG:
                    self.ekf_healthy = True
                else:
                    self.ekf_healthy = False
 



class WaypointListener(threading.Thread):

    def __init__(self, master):
        super(WaypointListener, self).__init__()
        self.master = master
        self.keep_running = True 

        self.current_wp = 0


    def run(self):

        while self.keep_running:

            #mission_current_msg = self.master.recv_match(type='MISSION_CURRENT', blocking=False)
            mission_current_msg = self.master.messages.get('MISSION_CURRENT', None)
            if mission_current_msg is not None:

                self.current_wp = mission_current_msg.seq


            time.sleep(0.1) 
