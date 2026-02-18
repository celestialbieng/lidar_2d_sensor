# UST-10LX Point Cloud Viewer

Simple program to visualize raw point cloud data from Hokuyo UST-10LX LiDAR.

## Features
- Connects directly to the LiDAR via TCP
- Displays raw point cloud data
- Option to view in Cartesian (x,y) or Polar (distance,angle) coordinates
- Simple console output

## Requirements
- CMake 3.7+
- C++11 compatible compiler
- Linux system (for socket programming)

## Building
```bash
mkdir build && cd build
cmake ..
make