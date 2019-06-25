#include <iostream>

#include <hsearch/types.h>
#include <hsearch/graph/directed_acyclic_graph.h>
#include <hsearch/graph/lattice.h>
#include <hsearch/space/action_space.h>
#include <hsearch/space/lattice_planning_space.h>

using namespace std;
using namespace hsearch;

void testDirectedAcyclicGraph(){
    cout<<"Testing DirectedAcyclicGraph\n";
    cout<<"============================\n";
    DirectedAcyclicGraph graph( 1 );
    graph.insertEdge( 1, 2 );
    graph.insertEdge( 2, 3 );
    graph.insertEdge( 1, 3 );

    cout<<"Number of vertices: "<<graph.m_vertices_map.size()<<"\n";

    auto succs = graph.Succs( 1 );
    cout<<"Successors of Node 1\n";
    for( auto el: succs ){
        cout<<el<<"\t";
    }
    cout<<"\n\n";
}

OccupancyGrid createOccupancyGrid(){
    double size_x=10, size_y=10, size_z=2;
    double res=0.05;
    double origin_x=0, origin_y=0, origin_z=0;
    double max_dist=1;
    return OccupancyGrid( size_x, size_y, size_z,
            res, origin_x, origin_y, origin_z, max_dist );
}

void testLattice(){
    cout<<"Testing LatticePlanningSpace\n";
    cout<<"============================\n";
    auto lattice_ptr = make_shared<Lattice>();

    auto grid = std::make_shared<OccupancyGrid>( createOccupancyGrid() );
    ActionSpacePtr action_space_ptr = make_shared<ActionSpace>( 3 );
    Action action1{ 1, 0, 0 };
    Action action2{ 0, 1, 0 };
    Action action3{ -1, 0, 0 };
    Action action4{ 0, -1, 0 };
    action_space_ptr->addAction( action1 );
    action_space_ptr->addAction( action2 );
    action_space_ptr->addAction( action3 );
    action_space_ptr->addAction( action4 );

    RobotState start = { 0, 0, 0 };
    cout<<"Start: \n";
    printVector( start );
    double res = 0.05;
    auto pspace_ptr = make_shared<LatticePlanningSpace>(
            grid, action_space_ptr, start );
    RobotState goal = {5, 5, 0};
    pspace_ptr->setGoal( goal );
    pspace_ptr->setGoalThresh( 0.1 );

    auto succs = pspace_ptr->Succs( start );
    cout<<"Succs: \n";
    for( auto a : succs )
        printVector( a );
    cout<<"\n\n";
}

int main(){
    testDirectedAcyclicGraph();
    testLattice();
}
