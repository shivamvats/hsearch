#ifndef LATTICE_H
#define LATTICE_H

#include "directed_acyclic_graph.h"
#include "hsearch/space/action_space.h"
#include "hsearch/types.h"

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

        virtual Nodes Succs( const Node );
        //virtual double EdgeCost( const Node, const Node);
    };

    using LatticePtr = std::shared_ptr<Lattice>;
} //namespace hsearch

#endif
