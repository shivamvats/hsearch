#include <chrono>
#include <hsearch/search/dijkstra.h>

namespace hsearch {
    Dijkstra::Dijkstra( LatticePlanningSpacePtr& pspace_ptr_  ) : LatticePlanner( pspace_ptr_ ){}

    bool Dijkstra::isGoal( const NodeId& node_id_ ) const {
        return m_pspace_ptr->isGoal( node_id_ );
    }

    bool Dijkstra::plan( double allocated_time_sec_, NodeIds& path_ ){
        auto start_time = std::chrono::high_resolution_clock::now();

        while( 1 ){
            auto curr_time = std::chrono::high_resolution_clock::now();
            if ( std::chrono::duration_cast<std::chrono::seconds>( curr_time - start_time ).count() > allocated_time_sec_ )
                return 1;
        }
    }

}
