#include "directed_acyclic_graph.h"

namespace hsearch {
    DirectedAcyclicGraph::DirectedAcyclicGraph( double edge_cost_ ){
        m_edge_cost = edge_cost_;
    }

    Nodes DirectedAcyclicGraph::Succs( Node s_ ){
        return m_adjacency_map[ s_ ];
    }

    double DirectedAcyclicGraph::EdgeCost( Node start_, Node end_ ){
        return m_edge_cost;
    }

    void DirectedAcyclicGraph::insertVertex( Node vertex_){
        m_vertices_map.emplace( vertex_, true );
    }

    void DirectedAcyclicGraph::insertEdge( Node start_, Node end_ ){
        insertVertex( start_ );
        insertVertex( end_ );
        m_vertices_map.emplace( end_, true );
        if (m_adjacency_map.count( start_ ) == 0 ){
            std::vector<Node> key{ end_ };
            m_adjacency_map[ start_ ] = key;
        }
        else {
            m_adjacency_map[ start_ ].push_back( end_ );
        }
    }
} //namespace hsearch
