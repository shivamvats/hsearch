#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "lattice_planner.h"

namespace hsearch {
    class Dijkstra : public LatticePlanner {
        public:
        Dijkstra( LatticePlanningSpacePtr& );

        virtual bool isGoal( NodeId );
        vritual bool plan( double, NodeIds& );
    };
}

#endif /* ifndef DIJKSTRA_H
