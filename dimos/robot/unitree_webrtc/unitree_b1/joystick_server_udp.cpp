/*****************************************************************
 UDP Joystick Control Server for Unitree B1 Robot
 With timeout protection and guaranteed packet boundaries
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <chrono>
#include <errno.h>

using namespace UNITREE_LEGGED_SDK;

// Joystick command structure received over network
struct NetworkJoystickCmd {
    float lx;  // left stick x (-1 to 1)
    float ly;  // left stick y (-1 to 1)
    float rx;  // right stick x (-1 to 1)
    float ry;  // right stick y (-1 to 1)
    uint16_t buttons;  // button states
    uint8_t mode;  // control mode
};

class JoystickServer {
public:
    JoystickServer(uint8_t level, int server_port) : 
        safe(LeggedType::B1),
        udp(level, 8090, "192.168.123.220", 8082),
        server_port_(server_port),
        running_(false) {
        udp.InitCmdData(cmd);
        memset(&joystick_cmd_, 0, sizeof(joystick_cmd_));
        joystick_cmd_.mode = 0;  // Start in idle mode
        last_packet_time_ = std::chrono::steady_clock::now();
    }

    void Start();
    void Stop();

private:
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void NetworkServerThread();
    void ParseJoystickCommand(const NetworkJoystickCmd& net_cmd);
    void CheckTimeout();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    
    NetworkJoystickCmd joystick_cmd_;
    std::mutex cmd_mutex_;
    
    int server_port_;
    int server_socket_;
    bool running_;
    std::thread server_thread_;
    
    // Client tracking for debug
    struct sockaddr_in last_client_addr_;
    bool has_client_ = false;
    
    // SAFETY: Timeout tracking
    std::chrono::steady_clock::time_point last_packet_time_;
    const int PACKET_TIMEOUT_MS = 100;  // Stop if no packet for 100ms
    
    float dt = 0.002;
    
    // Control parameters
    const float MAX_FORWARD_SPEED = 0.2f;   // m/s
    const float MAX_SIDE_SPEED = 0.2f;     // m/s  
    const float MAX_YAW_SPEED = 0.2f;      // rad/s
    const float MAX_BODY_HEIGHT = 0.1f;     // m
    const float MAX_EULER_ANGLE = 0.3f;     // rad
    const float DEADZONE = 0.0f;            // joystick deadzone
};

void JoystickServer::Start() {
    running_ = true;
    
    // Start network server thread
    server_thread_ = std::thread(&JoystickServer::NetworkServerThread, this);
    
    // Initialize environment
    InitEnvironment();
    
    // Start control loops
    LoopFunc loop_control("control_loop", dt, boost::bind(&JoystickServer::RobotControl, this));
    LoopFunc loop_udpSend("udp_send", dt, 3, boost::bind(&JoystickServer::UDPSend, this));
    LoopFunc loop_udpRecv("udp_recv", dt, 3, boost::bind(&JoystickServer::UDPRecv, this));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    std::cout << "UDP Joystick server started on port " << server_port_ << std::endl;
    std::cout << "Timeout protection: " << PACKET_TIMEOUT_MS << "ms" << std::endl;
    std::cout << "Expected packet size: 19 bytes" << std::endl;
    std::cout << "Robot control loops started" << std::endl;
    
    // Keep running
    while (running_) {
        sleep(1);
    }
}

void JoystickServer::Stop() {
    running_ = false;
    close(server_socket_);
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
}

void JoystickServer::NetworkServerThread() {
    // Create UDP socket
    server_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_socket_ < 0) {
        std::cerr << "Failed to create UDP socket" << std::endl;
        return;
    }
    
    // Allow socket reuse
    int opt = 1;
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Bind socket
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(server_port_);
    
    if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind UDP socket to port " << server_port_ << std::endl;
        close(server_socket_);
        return;
    }
    
    std::cout << "UDP server listening on port " << server_port_ << std::endl;
    std::cout << "Waiting for joystick packets..." << std::endl;
    
    NetworkJoystickCmd net_cmd;
    struct sockaddr_in client_addr;
    socklen_t client_len;
    
    while (running_) {
        client_len = sizeof(client_addr);
        
        // Receive UDP datagram (blocks until packet arrives)
        ssize_t bytes = recvfrom(server_socket_, &net_cmd, sizeof(net_cmd), 
                                  0, (struct sockaddr*)&client_addr, &client_len);
        
        if (bytes == 19) {
            // Perfect packet size from Python client
            if (!has_client_) {
                std::cout << "Client connected from " << inet_ntoa(client_addr.sin_addr) 
                          << ":" << ntohs(client_addr.sin_port) << std::endl;
                has_client_ = true;
                last_client_addr_ = client_addr;
            }
            ParseJoystickCommand(net_cmd);
        } else if (bytes == sizeof(NetworkJoystickCmd)) {
            // C++ client with padding (20 bytes)
            if (!has_client_) {
                std::cout << "C++ Client connected from " << inet_ntoa(client_addr.sin_addr) 
                          << ":" << ntohs(client_addr.sin_port) << std::endl;
                has_client_ = true;
                last_client_addr_ = client_addr;
            }
            ParseJoystickCommand(net_cmd);
        } else if (bytes > 0) {
            // Wrong packet size - ignore but log
            static int error_count = 0;
            if (error_count++ < 5) {  // Only log first 5 errors
                std::cerr << "Ignored packet with wrong size: " << bytes 
                          << " bytes (expected 19)" << std::endl;
            }
        }
        // Note: recvfrom returns -1 on error, which we ignore
    }
}

void JoystickServer::ParseJoystickCommand(const NetworkJoystickCmd& net_cmd) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    joystick_cmd_ = net_cmd;
    
    // SAFETY: Update timestamp for timeout tracking
    last_packet_time_ = std::chrono::steady_clock::now();
    
    // Apply deadzone to analog sticks
    if (fabs(joystick_cmd_.lx) < DEADZONE) joystick_cmd_.lx = 0;
    if (fabs(joystick_cmd_.ly) < DEADZONE) joystick_cmd_.ly = 0;
    if (fabs(joystick_cmd_.rx) < DEADZONE) joystick_cmd_.rx = 0;
    if (fabs(joystick_cmd_.ry) < DEADZONE) joystick_cmd_.ry = 0;
}

void JoystickServer::CheckTimeout() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_packet_time_).count();
    
    static bool timeout_printed = false;
    
    if (elapsed > PACKET_TIMEOUT_MS) {
        joystick_cmd_.lx = 0;
        joystick_cmd_.ly = 0;
        joystick_cmd_.rx = 0;
        joystick_cmd_.ry = 0;
        joystick_cmd_.buttons = 0;
        
        if (!timeout_printed) {
            std::cout << "SAFETY: Packet timeout - stopping movement!" << std::endl;
            timeout_printed = true;
        }
    } else {
        // Reset flag when packets resume
        if (timeout_printed) {
            std::cout << "Packets resumed - control restored" << std::endl;
            timeout_printed = false;
        }
    }
}

void JoystickServer::UDPRecv() {
    udp.Recv();
}

void JoystickServer::UDPSend() {
    udp.Send();
}

void JoystickServer::RobotControl() {
    udp.GetRecv(state);
    
    // SAFETY: Check for packet timeout
    NetworkJoystickCmd current_cmd;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        CheckTimeout();  // This may zero movement if timeout
        current_cmd = joystick_cmd_;
    }
    
    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;
    
    // Set mode from joystick
    cmd.mode = current_cmd.mode;
    
    // Map joystick to robot control based on mode
    switch (current_cmd.mode) {
        case 0:  // Idle
            // Robot stops
            break;
            
        case 1:  // Force stand with body control
            // Left stick controls body height and yaw
            cmd.bodyHeight = current_cmd.ly * MAX_BODY_HEIGHT;
            cmd.euler[2] = current_cmd.lx * MAX_EULER_ANGLE;
            
            // Right stick controls pitch and roll
            cmd.euler[1] = current_cmd.ry * MAX_EULER_ANGLE;
            cmd.euler[0] = current_cmd.rx * MAX_EULER_ANGLE;
            break;
            
        case 2:  // Walk mode
            cmd.velocity[0] = std::clamp(current_cmd.ly * MAX_FORWARD_SPEED, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
            cmd.yawSpeed = std::clamp(-current_cmd.lx * MAX_YAW_SPEED, -MAX_YAW_SPEED, MAX_YAW_SPEED);
            cmd.velocity[1] = std::clamp(-current_cmd.rx * MAX_SIDE_SPEED, -MAX_SIDE_SPEED, MAX_SIDE_SPEED);
            
            // Check button states for gait type
            if (current_cmd.buttons & 0x0001) {  // Button A
                cmd.gaitType = 0;  // Trot
            } else if (current_cmd.buttons & 0x0002) {  // Button B  
                cmd.gaitType = 1;  // Trot running
            } else if (current_cmd.buttons & 0x0004) {  // Button X
                cmd.gaitType = 2;  // Climb mode
            } else if (current_cmd.buttons & 0x0008) {  // Button Y
                cmd.gaitType = 3;  // Trot obstacle
            }
            break;
            
        case 5:  // Damping mode
        case 6:  // Recovery stand up
            break;
            
        default:
            cmd.mode = 0;  // Default to idle for safety
            break;
    }
    
    // Debug output
    static int counter = 0;
    if (counter++ % 500 == 0) {  // Print every second
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_packet_time_).count();
        
        std::cout << "Mode: " << (int)cmd.mode 
                  << " Vel: [" << cmd.velocity[0] << ", " << cmd.velocity[1] << "]"
                  << " Yaw: " << cmd.yawSpeed
                  << " Last packet: " << elapsed << "ms ago"
                  << " IMU: " << state.imu.rpy[2] << std::endl;
    }
    
    udp.SetSend(cmd);
}

// Signal handler for clean shutdown
JoystickServer* g_server = nullptr;

void signal_handler(int sig) {
    if (g_server) {
        std::cout << "\nShutting down server..." << std::endl;
        g_server->Stop();
    }
    exit(0);
}

int main(int argc, char* argv[]) {
    int port = 9090;  // Default port
    
    if (argc > 1) {
        port = atoi(argv[1]);
    }
    
    std::cout << "UDP Unitree B1 Joystick Control Server" << std::endl;
    std::cout << "Communication level: HIGH-level" << std::endl;
    std::cout << "Protocol: UDP (datagram)" << std::endl;
    std::cout << "Server port: " << port << std::endl;
    std::cout << "Packet size: 19 bytes (Python) or 20 bytes (C++)" << std::endl;
    std::cout << "Update rate: 50Hz expected" << std::endl;
    std::cout << "WARNING: Make sure the robot is standing on the ground." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    JoystickServer server(HIGHLEVEL, port);
    g_server = &server;
    
    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    server.Start();
    
    return 0;
}