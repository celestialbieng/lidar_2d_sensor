#include <zmq.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <fstream>
#include <sstream>
#include "UST10LX.h"

void loadConfig(std::string& lidar_ip, uint16_t& lidar_port, int& coordinate_choice, std::string& zmq_ip, uint16_t& zmq_port, float& angle_offset) {
    std::ifstream config_file("../config.ini");
    if (!config_file.is_open()) {
        std::cerr << "Error: Could not open config.ini. Using default values." << std::endl;
        return;
    }
    std::string line;
    while (std::getline(config_file, line)) {
        std::istringstream iss(line);
        std::string key, value;
        if (std::getline(iss, key, '=') && std::getline(iss, value)) {
            if (key == "lidar_ip") lidar_ip = value;
            else if (key == "lidar_port") lidar_port = std::stoi(value);
            else if (key == "coordinate_system") coordinate_choice = std::stoi(value);
            else if (key == "zmq_ip") zmq_ip = value;
            else if (key == "zmq_port") zmq_port = std::stoi(value);
            else if (key == "angle_offset") angle_offset = std::stof(value);
        }
    }
    std::cout << "Loaded config: lidar_ip=" << lidar_ip << ", lidar_port=" << lidar_port
              << ", coordinate_system=" << coordinate_choice << ", zmq_ip=" << zmq_ip
              << ", zmq_port=" << zmq_port << ", angle_offset=" << angle_offset << std::endl;
}

void plotCartesian(const std::vector<DataPoint>& points) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Cartesian Coordinates (x, y) in meters:" << std::endl;
    for(const auto& p : points) {
        if(p.distance == UST10LX::invalidDistance) continue;
        float x = p.distance * 0.001f * cos(p.angle);
        float y = p.distance * 0.001f * sin(p.angle);
        std::cout << "(" << std::setw(6) << x << ", " << std::setw(6) << y << ")\n";
    }
}

void plotPolar(const std::vector<DataPoint>& points) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Polar Coordinates (distance in meters, angle in degrees):" << std::endl;
    for(const auto& p : points) {
        if(p.distance == UST10LX::invalidDistance) continue;
        float angle_deg = p.angle * 180.0f / M_PI;
        while(angle_deg > 180) angle_deg -= 360;
        while(angle_deg < -180) angle_deg += 360;
        std::cout << "(" << std::setw(5) << p.distance * 0.001f
                  << ", " << std::setw(6) << angle_deg << "°)\n";
    }
}

int main() {
    // Load configuration
    std::string lidar_ip = "192.168.0.10";
    uint16_t lidar_port = 10940;
    int coordinate_choice = 1; // 1 for Cartesian, 2 for Polar
    std::string zmq_ip = "192.168.1.10";
    uint16_t zmq_port = 5000;
    float angle_offset = 180.0f; // Default to 0 to align 0° with positive x-axis
    loadConfig(lidar_ip, lidar_port, coordinate_choice, zmq_ip, zmq_port, angle_offset);

    // Initialize ZeroMQ
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.set(zmq::sockopt::sndhwm, 1); // Keep only latest message
    publisher.set(zmq::sockopt::sndbuf, 131072); // 128KB send buffer
    publisher.set(zmq::sockopt::conflate, 1); // Drop old messages
    publisher.set(zmq::sockopt::immediate, 1); // Don't buffer if subscriber not ready
    std::string zmq_address = "tcp://" + zmq_ip + ":" + std::to_string(zmq_port);
    publisher.bind(zmq_address);

    UST10LX lidar(angle_offset);

    std::cout << "Connecting to LiDAR at " << lidar_ip << ":" << lidar_port << "..." << std::endl;
    if(!lidar.connect(lidar_ip, lidar_port)) {
        std::cerr << "Failed to connect to LiDAR!" << std::endl;
        return 1;
    }

    // Validate coordinate choice
    if(coordinate_choice != 1 && coordinate_choice != 2) {
        std::cerr << "Invalid coordinate system in config. Using Cartesian." << std::endl;
        coordinate_choice = 1;
    }

    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &param)) {
        std::cerr << "Warning: Failed to set real-time priority ("
                  << strerror(errno) << "). Running with normal priority.\n";
        if(errno == EPERM) {
            std::cerr << "Try running with sudo for real-time priority.\n";
        }
    }

    std::cout << "Starting scan..." << std::endl;
    auto scan_interval = std::chrono::milliseconds(40); // 40ms for faster updates
    auto next_scan = std::chrono::steady_clock::now();

    while(true) {
        auto scan_start = std::chrono::steady_clock::now();
        next_scan += scan_interval;

        if(lidar.scan()) {
            const auto& points = lidar.getDataPoints();
            
            // Print point cloud
            if(coordinate_choice == 1) {
                plotCartesian(points);
            } else {
                plotPolar(points);
            }

            // Prepare binary data for points
            std::vector<uint8_t> binary_data;
            binary_data.reserve(points.size() * 8 + 4); // 8 bytes per point + 4 bytes for count
            uint32_t point_count = 0;
            for(const auto& p : points) {
                if(p.distance == UST10LX::invalidDistance) continue;
                point_count++;
            }
            // Pack point count first (4 bytes)
            uint8_t* count_bytes = reinterpret_cast<uint8_t*>(&point_count);
            for(int i = 0; i < 4; ++i) binary_data.push_back(count_bytes[i]);
            // Pack points
            for(const auto& p : points) {
                if(p.distance == UST10LX::invalidDistance) continue;
                float x = p.distance * 0.001f * cos(p.angle);
                float y = p.distance * 0.001f * sin(p.angle);
                uint8_t* x_bytes = reinterpret_cast<uint8_t*>(&x);
                uint8_t* y_bytes = reinterpret_cast<uint8_t*>(&y);
                for(int i = 0; i < 4; ++i) binary_data.push_back(x_bytes[i]);
                for(int i = 0; i < 4; ++i) binary_data.push_back(y_bytes[i]);
            }

            // Send binary data via ZeroMQ
            zmq::message_t zmq_msg(binary_data.size());
            memcpy(zmq_msg.data(), binary_data.data(), binary_data.size());
            publisher.send(zmq_msg, zmq::send_flags::none);

            // Log timing
            auto scan_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - scan_start).count();
            std::cout << "Scan and send time: " << scan_duration / 1000.0 << " ms" << std::endl;
        }

        auto sleep_time = next_scan - std::chrono::steady_clock::now();
        if(sleep_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    return 0;
}