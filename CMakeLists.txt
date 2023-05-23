cmake_minimum_required(VERSION 3.10)

project(vbs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Check if the compiler supports C++20
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)

# If C++20 is not supported, fallback to C++11
if (NOT COMPILER_SUPPORTS_CXX20)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} does not support C++20. Fallback to C++11.")
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Ofast -flto=auto -std=c++20 -Wall -Wextra -Wpedantic -Wformat=2 -Wnull-dereference -D_USE_MATH_DEFINES -Wno-sign-compare")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

include_directories(include)
add_executable(vbs src/main.cpp src/environment.cpp src/solver.cpp src/ConfigParser.cpp)

# Find the SFML package
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
# Link the SFML graphics library
target_link_libraries(vbs PRIVATE sfml-graphics)