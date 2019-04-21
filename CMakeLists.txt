cmake_minimum_required(VERSION 3.6)
project(hsearch)
set(CMAKE_CXX_STANDARD 11)

include_directories(include)
set(SOURCE_FILES src/graph/directed_acyclic_graph.cpp
    src/graph/lattice.cpp
    src/space/action_space.cpp
    src/space/robot_state.cpp
    src/space/lattice_planning_space.cpp
    )
add_library(hsearch ${SOURCE_FILES})

add_executable(test test/graph.cpp)
target_link_libraries(test hsearch)
