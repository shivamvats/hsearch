#ifndef DIRECTED_ACYCLIC_GRAPH_H
#define DIRECTED_ACYCLIC_GRAPH_H

#include <unordered_map>
#include "types.h"

namespace hsearch {
    class DirectedAcyclicGraph {
        public:
        DirectedAcyclicGraph( double );
        virtual Nodes Succs( Node );
        virtual double EdgeCost( Node, Node );

        void insertVertex( Node );
        void insertEdge( Node, Node );

        std::unordered_map<size_t, std::vector<size_t>> m_adjacency_map;
        std::unordered_map<size_t, bool> m_vertices_map;
        double m_edge_cost;
    };
} //namespace hsearch

#endif /* ifndef DIRECTED_ACYCLIC_GRAPH_H */
