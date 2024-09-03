#pragma once

// Extern declarations for global variables
extern bool override_flag;
extern int throttle_val;
extern int steering_val;
extern int server_fd;
extern bool reset_override;

// Global variables for GPS spoofing
extern bool gps_spoofing_flag;
extern float spoofed_x_offset;
extern float spoofed_y_offset;

// Global variables for GYRO spoofing
extern bool gyro_spoofing_flag;
extern float spoofed_roll_offset;
extern float spoofed_pitch_offset;
extern float spoofed_yaw_offset;


// Function declarations
void* run_tcp_server_safesim(void* arg);