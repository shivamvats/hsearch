#include <iostream>
#include "graph/directed_acyclic_graph.h"
#include "graph/lattice.h"
#include "space/action_space.h"
#include "space/lattice_planning_space.h"

using namespace std;

void testDirectedAcyclicGraph(){
    cout<<"Testing DirectedAcyclicGraph\n";
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
    cout<<"\n";
}

void testLattice(){
    cout<<"Testing LatticePlanningSpace\n";
    auto lattice_ptr = make_shared<hsearch::Lattice>();
    hsearch::ActionSpacePtr action_space_ptr = make_shared<hsearch::ActionSpace>( 3 );
    hsearch::Action action1{ 1, 1, 1 };
    hsearch::Action action2{ 2, 2, 2 };
    action_space_ptr->addAction( action1 );
    action_space_ptr->addAction( action2 );

    hsearch::RobotState start( std::vector<double>{ 0, 0, 0 } );
    cout<<"Start: \n";
    start.print();
    auto lattice_planning_space_ptr = make_shared<hsearch::LatticePlanningSpace>(
                                            action_space_ptr, start );
    auto succs = lattice_planning_space_ptr->Succs( start );
    cout<<"Succs: \n";
    for( auto a : succs )
        a.print();

}

int main(){
    testDirectedAcyclicGraph();
    testLattice();
}
