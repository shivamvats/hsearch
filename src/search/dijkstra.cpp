#include <chrono>
#include <hsearch/search/dijkstra.h>

namespace hsearch {
    Dijkstra::Dijkstra( LatticePlanningSpacePtr& pspace_ptr_  ) : LatticePlanner( pspace_ptr_ ){}

    bool Dijkstra::isGoal( const NodeId& node_id_ ) const {
        return m_pspace_ptr->isGoal( node_id_ );
    }

    bool Dijkstra::plan( double allocated_time_sec_, NodeIds& path_ ){
        NodeId start_node = m_pspace_ptr->robotStateToNodeId( m_pspace_ptr->m_start );

        auto start_time = std::chrono::high_resolution_clock::now();

        //while( !isGoal( curr_node ) || !m_open.empty() ){
        //    auto curr_time = std::chrono::high_resolution_clock::now();
        //    if( std::chrono::duration_cast<std::chrono::seconds>
        //            ( curr_time - start_time ).count() > allocated_time_sec_ )
        //        return 1;
        //}
    }

    Dijkstra::SearchStatePtr Dijkstra::getSearchStatePtr( const NodeId& node_id_ ){
    if ( m_node_id_to_search_state.find( node_id_ ) == m_node_id_to_search_state.end() ) {
        m_node_id_to_search_state[ node_id_ ] = std::make_shared<SearchState>();
        m_node_id_to_search_state[ node_id_ ]->bp = nullptr;
        m_node_id_to_search_state[ node_id_ ]->call_number = 0;
        m_node_id_to_search_state[ node_id_ ]->node_id = node_id_;
    }

    return m_node_id_to_search_state[ node_id_ ];
    }

} //namespace hsearch
