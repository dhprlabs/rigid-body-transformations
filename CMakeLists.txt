cmake_minimum_required(VERSION 3.10)
project(rigid-body-transformations)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libs/eigen-3.4.0
)

# Create executables
add_executable(Transforms 
    Transforms.cpp
    src/RigidBodyTransforms.cpp        
)