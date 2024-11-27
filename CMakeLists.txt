cmake_minimum_required(VERSION 3.10)

project(vbm)

# set the C++ standard
set(CMAKE_CXX_STANDARD 20)
# set the C++ compiler version
set(CMAKE_CXX_COMPILER "g++-11")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the build type to Release by default
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall -Wextra -Wnull-dereference -Wpedantic -flto=auto -std=c++20 -D_USE_MATH_DEFINES")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -fsanitize=address -std=c++20 -D_USE_MATH_DEFINES")

# Set the default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

include_directories(include)
add_executable(vbm src/main.cpp src/environment.cpp src/solver.cpp src/parser.cpp)

# Find the SFML package
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
# Link the SFML graphics library
target_link_libraries(vbm PRIVATE sfml-graphics)
