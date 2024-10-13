cmake_minimum_required(VERSION 3.2)

# Set the project name
project(jetson-bno055)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(lib)
add_subdirectory(examples)