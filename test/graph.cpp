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
    hsearch::DirectedAcyclicGraph graph( 1 );
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
    auto lattice_ptr = make_shared<hsearch::Lattice>();

    auto grid = std::make_shared<OccupancyGrid>( createOccupancyGrid() );
    hsearch::ActionSpacePtr action_space_ptr = make_shared<hsearch::ActionSpace>( 3 );
    hsearch::Action action1{ 1, 1, 1 };
    hsearch::Action action2{ 2, 2, 2 };
    action_space_ptr->addAction( action1 );
    action_space_ptr->addAction( action2 );

    hsearch::RobotState start( std::vector<double>{ 0, 0, 0 } );
    cout<<"Start: \n";
    printVector( start );
    auto lattice_planning_space_ptr = make_shared<hsearch::LatticePlanningSpace>(
            grid, action_space_ptr, start );
    auto succs = lattice_planning_space_ptr->Succs( start );
    cout<<"Succs: \n";
    for( auto a : succs )
        printVector( a );
    cout<<"\n\n";
}

int main(){
    testDirectedAcyclicGraph();
    testLattice();
}
