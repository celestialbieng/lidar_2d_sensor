cmake_minimum_required(VERSION 3.12)
project(lidar_2d_sensor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Threads package
find_package(Threads REQUIRED)

# Find ZeroMQ package
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Add executable
add_executable(lidar_2d_sensor
    src/main.cpp
    src/UST10LX.cpp
)

# Include directories
target_include_directories(lidar_2d_sensor PRIVATE 
    include
    ${ZMQ_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(lidar_2d_sensor PRIVATE 
    Threads::Threads
    ${ZMQ_LIBRARIES}
)
