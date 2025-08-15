cmake_minimum_required(VERSION 3.28) # Keep whatever version was generated
project(ConsoleTest) # This is the name of your new project

set(CMAKE_CXX_STANDARD 20)

# Tell CMake to include both of your files in the executable
add_executable(ConsoleTest main.cpp Graph.h
        Graph.cpp
        Graph.h)