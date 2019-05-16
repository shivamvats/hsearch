cmake_minimum_required(VERSION 3.6)
project(hsearch)
set(CMAKE_CXX_STANDARD 11)

find_package( Eigen3 REQUIRED )
list( APPEND CMAKE_PREFIX_PATH "/Users/aries/local" )
find_package( smpl CONFIG REQUIRED )
message( STATUS ${Eigen3_INCLUDE_DIRS} )

set(SOURCE_FILES
    src/graph/directed_acyclic_graph.cpp
    src/graph/lattice.cpp
    src/space/action_space.cpp
    src/space/lattice_planning_space.cpp
    src/search/lattice_planner.cpp
    src/search/dijkstra.cpp
    )

add_library( hsearch ${SOURCE_FILES} )
target_include_directories( hsearch PUBLIC
    include
    ${Eigen3_INCLUDE_DIRS}
    ${smpl_INCLUDE_DIRS})
target_link_libraries( hsearch
    ${Eigen3_LIBRARIES}
    ${smpl_LIBRARIES})

add_executable(test test/graph.cpp)
target_link_libraries(test hsearch )
