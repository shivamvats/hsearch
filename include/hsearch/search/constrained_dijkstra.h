#ifndef CONSTRAINED_DIJKSTRA_H
#define CONSTRAINED_DIJKSTRA_H

#include "dijkstra.h"
#include <hsearch/space/constrained_two_dim_grid_space.h>

namespace hsearch {
    class ConstrainedDijkstra : public Dijkstra {
        public:
        ConstrainedDijkstra( LatticePlanningSpacePtr& );
        ~ConstrainedDijkstra(){};

        void reinit() override;
        virtual bool plan( const double, NodeIds& ) override;

        public:
        OpenList m_suspended;
    };
}

#endif
