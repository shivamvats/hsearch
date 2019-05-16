#include <hsearch/search/dijkstra.h>

namespace hsearch {
    Dijkstra::Dijkstra( LatticePlanningSpacePtr& pspace_ptr_  ) : LatticePlanner( pspace_ptr_ ){}

    bool Dijkstra::isGoal( const NodeId& node_id_ ) const {
        return m_pspace_ptr->isGoal( node_id_ );
    }
}
