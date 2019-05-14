#include <hsearch/search/lattice_planner.h>

namespace hsearch {
    LatticePlanner::LatticePlanner( LatticePlanningSpacePtr& pspace_ptr_ ) : m_pspace_ptr( pspace_ptr_ ){}

    LatticePlanner::setStart( NodeId start_ ) : m_start( start_ ){}
}
