# UST-10LX Point Cloud Viewer

Simple program to visualize and publish raw point cloud data from Hokuyo UST-10LX LiDAR.

## Features
- Connects directly to the LiDAR via TCP
- Displays raw point cloud data in the console
- Option to view in Cartesian (x,y) or Polar (distance,angle) coordinates
- Publishes point cloud data via ZeroMQ with Base64 encoding for efficient streaming
- Configuration via `config.ini` for LiDAR IP, port, coordinate system, and ZeroMQ settings
- Companion Python script (`lidar_viewer.py`) for real-time visualization using PyQt5 with a dark theme and heatmap

## Requirements
- CMake 3.12+
- C++17 compatible compiler
- Linux system (for socket programming and real-time scheduling)
- ZeroMQ (`libzmq3-dev`) for data publishing
- Python 3.6+ with `pyqt5`, `pyzmq`, and `numpy` for the visualization script
- Network interface configured (e.g., `eth0` for LiDAR/ZeroMQ communication)

## File Structure
```
lidar_2d_sensor/
├── include/
│   ├── DataPoint.h
│   └── UST10LX.h
├── src/
│   ├── main.cpp
│   └── UST10LX.cpp
├── CMakeLists.txt
├── config.ini
├── README.md
```

## Building
1. Ensure dependencies are installed:
   ```bash
   sudo apt-get install libzmq3-dev
   ```
2. Configure the network interface (if required):
   ```bash
   sudo ip link set eth0 up
   ```
3. Build the project:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

## Configuration
Create a `config.ini` file in the same directory as the executable with the following content:
```ini
[DEFAULT]
lidar_ip = 192.168.0.10
lidar_port = 10940
coordinate_system = 1
zmq_ip = 192.168.55.1
zmq_port = 5000
```
- `lidar_ip` and `lidar_port`: IP address and port of the Hokuyo UST-10LX LiDAR.
- `coordinate_system`: 1 for Cartesian (x,y), 2 for Polar (distance,angle) in console output.
- `zmq_ip` and `zmq_port`: IP address and port for ZeroMQ publishing.

## Running
1. Run the C++ program to connect to the LiDAR and publish data:
   ```bash
   ./lidar_2d_sensor
   ```
   - The program will print point cloud data to the console and publish it via ZeroMQ to `tcp://192.168.55.1:5000` (configurable in `config.ini`).
   - For real-time scheduling, you may need to run with `sudo`:
     ```bash
     sudo ./lidar_2d_sensor
     ```

2. Run the Python visualization script on a machine with network access to `192.168.55.1:5000`:
   ```bash
   pip install pyqt5 pyzmq numpy
   python3 lidar_viewer.py
   ```
   - The script reads `config.ini` for ZeroMQ settings and displays a real-time point cloud with a dark theme and heatmap (blue for closer points, red for farther).

## Notes
- The C++ program uses Base64 encoding for efficient data transmission over ZeroMQ.
- The Python visualization script requires `config.ini` in its directory with `zmq_ip` and `zmq_port` matching the C++ program's settings.
- Ensure the network interface (`eth0`) is up and configured for communication with the LiDAR and ZeroMQ endpoints.
- If the LiDAR or ZeroMQ connection fails, check the IP addresses, ports, and network connectivity (`ping 192.168.0.10` or `ping 192.168.55.1`).
