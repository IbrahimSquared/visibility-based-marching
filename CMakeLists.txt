cmake_minimum_required(VERSION 3.10)

project(vbs)

# Set the compiler to use
set(CMAKE_CXX_COMPILER C:/Workdir/Programs/msys64/mingw64/bin/g++.exe)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Ofast -flto=auto -std=c++20 -Wall -Wextra -Wpedantic -Wformat=2 -Wnull-dereference -D_USE_MATH_DEFINES -Wno-sign-compare")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

include_directories(include)
include_directories(C:/Workdir/Programs/msys64/mingw64/include)
add_executable(vbs src/main.cpp src/environment.cpp src/solver.cpp src/ConfigParser.cpp)

# Find the SFML package
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
# Link the SFML graphics library
target_link_libraries(vbs PRIVATE sfml-graphics)