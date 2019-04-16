cmake_minimum_required(VERSION 3.6)
project(hsearch)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES src/graph.cpp
                 src/env.cpp
                 src/astar.cpp)
add_library(heuristic_search ${SOURCE_FILES})
