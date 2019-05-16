#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "lattice_planner.h"

namespace hsearch {
    class Dijkstra : public LatticePlanner {
        public:
        Dijkstra( LatticePlanningSpacePtr& );

        virtual bool isGoal( const NodeId& ) const;
        virtual bool plan( double, NodeIds& );
    };
}

#endif
