# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(robot_comms)

set(CMAKE_CXX_STANDARD 17)

# Robot binary (Jetson)
add_executable(robot
    main.cpp
    robot_comms.cpp
)
target_compile_options(robot PRIVATE -Wall -O2)

# Base binary
add_executable(base
    base_main.cpp
    base_comms.cpp
)
target_compile_options(base PRIVATE -Wall -O2)