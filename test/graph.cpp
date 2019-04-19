#include <iostream>
#include "directed_acyclic_graph.h"

using namespace std;

int main(){
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
