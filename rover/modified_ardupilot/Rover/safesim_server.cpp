// safesim_server.cpp
#include "safesim_server.h"

#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <iterator>
#include <string>

// Global variables for override
bool override_flag = false;
int throttle_val = 0;
int steering_val = 0;
bool reset_override = false;

// Global variables for GPS spoofing
bool gps_spoofing_flag = false;
float spoofed_x_offset = 0.0;
float spoofed_y_offset = 0.0;

// Global variables for GYRO spoofing
bool gyro_spoofing_flag = false;
float spoofed_roll_offset = 0.0;
float spoofed_pitch_offset = 0.0;
float spoofed_yaw_offset = 0.0;

// Global variable for server
int server_fd = -1; 




static void* handle_client(void* arg) {
    int client_socket = *(int*)arg;
    char buffer[1024] = {0};

    while (true) {
        memset(buffer, 0, sizeof(buffer));
        int valread = read(client_socket, buffer, 1024);
        if (valread > 0) {
            std::string command(buffer);
            std::istringstream iss(command);
            std::vector<std::string> tokens(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());


            if (tokens[0] == "override") { // control override handler
                if(tokens.size() == 4 && tokens[1] == "start") {
                    throttle_val = std::stof(tokens[2]);
                    steering_val = std::stof(tokens[3]);
                    override_flag = true;
                } else if(tokens.size() == 2 && tokens[1] == "stop") {
                    override_flag = false;
                    reset_override = true;
                } 
            } else if (tokens[0] == "gps") { // gps spoofing handler
                if(tokens.size() == 4 && tokens[1] == "start") {
                    spoofed_x_offset = std::stof(tokens[2]);
                    spoofed_y_offset = std::stof(tokens[3]);
                    gps_spoofing_flag = true;
                    
                } else if(tokens.size() == 2 && tokens[1] == "stop") {
                    gps_spoofing_flag = false;
                } 
            } else if (tokens[0] == "gyro") { // gyro spoofing handler
                if(tokens.size() == 3 && tokens[1] == "start") {
                    
                    spoofed_yaw_offset = std::stof(tokens[2]);
                    gyro_spoofing_flag = true;
                } else if(tokens.size() == 2 && tokens[1] == "stop") {
                    gyro_spoofing_flag = false;
                } 
            } else if (tokens[0] == "terminate") { // termination handler

                close(client_socket);
                // Close the server socket to terminate the server loop
                close(server_fd);
                server_fd = -1;
                pthread_exit(NULL);
            }
            
        } else {
            close(client_socket);
            break;
        }
    }
    return nullptr;
}

void* run_tcp_server_safesim(void* arg) {


    // Removed the local declaration of server_fd
    int new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        pthread_exit(NULL);
    }

    int optval = 1;
    socklen_t optlen = sizeof(optval);

    if(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &optval, optlen) < 0) {
        
        close(server_fd);
        pthread_exit(NULL);
    }

    if(setsockopt(server_fd, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
      
        close(server_fd);
        pthread_exit(NULL);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(14551);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
     
        pthread_exit(NULL);
    }

    if (listen(server_fd, 3) < 0) {
    
        pthread_exit(NULL);
    }

    while(true) {
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            // If server_fd is -1, it means the server has been signaled to stop
            if(server_fd == -1) {
                pthread_exit(NULL);
            }
            perror("accept");
            pthread_exit(NULL);
        }
        pthread_t client_thread;
        pthread_create(&client_thread, NULL, handle_client, &new_socket);
    }
    return nullptr;
}

