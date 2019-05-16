#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include <hsearch/space/lattice_planning_space.h>
#include <hsearch/types.h>

namespace hsearch {
    /**
     * Base class for all planners.
     */
    class LatticePlanner {
        public:
        LatticePlanner( LatticePlanningSpacePtr& );
        bool setStart( NodeId& );
        virtual bool isGoal ( const NodeId& ) const = 0;
        virtual bool plan( const double allocated_time_sec_, NodeIds& path_ ) = 0;

        LatticePlanningSpacePtr m_pspace_ptr;
        NodeId m_start;
    };
}

#endif
