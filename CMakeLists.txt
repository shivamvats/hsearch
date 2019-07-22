cmake_minimum_required(VERSION 3.6)
project(hsearch)
set(CMAKE_CXX_STANDARD 11)

find_package( Eigen3 REQUIRED )
#find_package(PythonLibs 2.7 EXACT REQUIRED)
find_package( OpenCV 3 REQUIRED )
find_package( Boost REQUIRED )
list( APPEND CMAKE_PREFIX_PATH "/Users/aries/local" )
find_package( smpl CONFIG REQUIRED )

set(SOURCE_FILES
    src/graph/directed_acyclic_graph.cpp
    src/graph/lattice.cpp
    src/collision_checking/two_dim_grid_collision_checker.cpp
    src/space/action_space.cpp
    src/space/lattice_planning_space.cpp
    src/space/two_dim_grid_space.cpp
    src/space/constrained_two_dim_grid_space.cpp
    src/search/lattice_planner.cpp
    src/search/dijkstra.cpp
    src/viz/visualizer.cpp
    )

add_library( hsearch ${SOURCE_FILES} )
target_include_directories( hsearch PUBLIC
    include
    ${Eigen3_INCLUDE_DIRS}
    ${smpl_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )
target_link_libraries( hsearch
    ${Eigen3_LIBRARIES}
    ${smpl_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    )

#add_executable( viz_matplot src/viz/viz_matplot.cpp )
#target_include_directories( viz_matplot PRIVATE ${PYTHON_INCLUDE_DIRS} )
#target_link_libraries( viz_matplot hsearch ${PYTHON_LIBRARIES} )

#add_executable(test_graph test/graph.cpp)
#target_link_libraries(test_graph hsearch )

#add_executable( test_dijkstra test/dijkstra.cpp )
#target_link_libraries( test_dijkstra hsearch )

add_executable( test_viz test/visualizer.cpp )
target_link_libraries( test_viz hsearch )

add_executable( test_two_dim_grid test/two_dim_grid.cpp )
target_link_libraries( test_two_dim_grid hsearch )

add_executable( test_constrained_two_dim_grid test/constrained_two_dim_grid.cpp )
target_link_libraries( test_constrained_two_dim_grid hsearch )

add_executable( curve_similarity src/curve_similarity.cpp )
target_link_libraries( curve_similarity hsearch )
