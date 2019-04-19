#ifndef LATTICE_H
#define LATTICE_H

#include "directed_acyclic_graph.h"
#include "types.h"

namespace hsearch {
    /**
    * The Lattice class supports two public methods:
    *  - Succs(*)
    *  - EdgeCost(*)
    * that are required to traverse the graph.
    */
    class Lattice : public DirectedAcyclicGraph {
        public:
        Lattice();

        virtual Nodes Succs( const Node, const Actions ) = 0;
        virtual float EdgeCost( const Node, const Node ) = 0;

        bool setActions( const Actions );

        Actions m_actions;
    };
} //namespace hsearch

#endif
