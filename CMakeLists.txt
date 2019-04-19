cmake_minimum_required(VERSION 3.6)
project(hsearch)
set(CMAKE_CXX_STANDARD 11)

include_directories(include)
set(SOURCE_FILES src/directed_acyclic_graph.cpp
    )
add_library(heuristic_search ${SOURCE_FILES})

add_executable(test test/graph.cpp)
target_link_libraries(test heuristic_search)
