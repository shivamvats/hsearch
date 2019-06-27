#include <chrono>
#include <hsearch/search/dijkstra.h>
#include <hsearch/types.h>

bool DEBUG = 1;

namespace hsearch {
    Dijkstra::Dijkstra( LatticePlanningSpacePtr& pspace_ptr_  ) : LatticePlanner( pspace_ptr_ ){
        m_start_node_id = m_pspace_ptr->robotStateToNodeId( m_pspace_ptr->m_start );
    }

    Dijkstra::~Dijkstra(){
        for( auto& it : m_node_id_to_search_state ){
            delete it.second;
        }
    }

    bool Dijkstra::isGoal( const NodeId& node_id_ ) const {
        return m_pspace_ptr->isGoal( node_id_ );
    }

    bool Dijkstra::plan( double allocated_time_sec_, NodeIds& path_ ){
        SearchStatePtr curr_state_ptr = getSearchStatePtr( m_start_node_id );
        curr_state_ptr->g = 0;
        m_open.push( curr_state_ptr );

        auto start_time = std::chrono::high_resolution_clock::now();

        while(!m_open.empty() ){
            if( isGoal( curr_state_ptr->node_id ) ){
                std::cout<<"Goal found. Cost: "<<curr_state_ptr->g<<"\n";
                return 0;
            }

            auto curr_time = std::chrono::high_resolution_clock::now();
            if( std::chrono::duration_cast<std::chrono::seconds>
                    ( curr_time - start_time ).count() > allocated_time_sec_ ){
                std::cout<<"Time out\n";
                return 1;
            }

            curr_state_ptr = m_open.min();
            m_open.pop();
            if( m_closed.count( curr_state_ptr->node_id  ) )
                continue;
            m_closed.insert( curr_state_ptr->node_id );

            NodeIds succs = m_pspace_ptr->Succs( curr_state_ptr->node_id );

            if( ::DEBUG ){
                std::cout<<"Current state: ";
                printVector(m_pspace_ptr->nodeIdToRobotState(curr_state_ptr->node_id));
                std::cout<<"Current state g: "<<curr_state_ptr->g<<"\n";
                std::cout<<"Open: "<<m_open.size()<<"\n";
                std::cout<<"Succs: "<<succs.size()<<"\n";
            }
            for( NodeId& succ: succs ){
                auto search_state_ptr = getSearchStatePtr( succ );
                int new_g = curr_state_ptr->g + 1;
                if( ::DEBUG ){
                    std::cout<<"Succ: ";
                    printVector(m_pspace_ptr->nodeIdToRobotState(search_state_ptr->node_id));
                    std::cout<<"Old g: "<<search_state_ptr->g<<"\n";
                    std::cout<<"New g: "<<new_g<<"\n";
                }
                if( new_g < search_state_ptr->g ){
                    search_state_ptr->g = new_g;
                    search_state_ptr->bp = curr_state_ptr;
                    m_open.push( search_state_ptr );
                }
            }
        }
        std::cout<<"Ran out of nodes.\n";
        return 1;
    }

    Dijkstra::SearchStatePtr Dijkstra::getSearchStatePtr( const NodeId& node_id_ ){
    if ( m_node_id_to_search_state.find( node_id_ ) == m_node_id_to_search_state.end() ) {
        m_node_id_to_search_state[ node_id_ ] = new SearchState;
        m_node_id_to_search_state[ node_id_ ]->bp = nullptr;
        m_node_id_to_search_state[ node_id_ ]->call_number = 0;
        m_node_id_to_search_state[ node_id_ ]->node_id = node_id_;
        m_node_id_to_search_state[ node_id_ ]->g = INT_MAX;
    }

    return m_node_id_to_search_state[ node_id_ ];
    }

} //namespace hsearch
