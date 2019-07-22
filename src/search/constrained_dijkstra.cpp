#include <hsearch/search/constrained_dijkstra.h>

static bool DEBUG = 1;

namespace hsearch {
    ConstrainedDijkstra::ConstrainedDijkstra(
            LatticePlanningSpacePtr& pspace_ptr_ ) :
            Dijkstra( pspace_ptr_ ) {}

    void ConstrainedDijkstra::clear(){
        m_open.clear();
        m_closed.clear();
        m_suspended.clear();
    }

    bool ConstrainedDijkstra::plan( double allocated_time_sec_, NodeIds& path_ ){
        std::cout<<"hi\n";
        SearchStatePtr curr_state_ptr = getSearchStatePtr( m_start_node_id );
        curr_state_ptr->g = 0;
        m_open.push( curr_state_ptr );

        auto start_time = std::chrono::high_resolution_clock::now();
        auto constrained_pspace_ptr = std::dynamic_pointer_cast<ConstrainedTwoDimGridSpace>(m_pspace_ptr);

        while(!m_open.empty() ){
            if( isGoal( curr_state_ptr->node_id ) ){
                std::cout<<"Goal found. Cost: "<<curr_state_ptr->g<<"\n";
                extractPath( curr_state_ptr, path_ );
                return 1;
            }

            auto curr_time = std::chrono::high_resolution_clock::now();
            if( std::chrono::duration_cast<std::chrono::seconds>
                    ( curr_time - start_time ).count() > allocated_time_sec_ ){
                std::cout<<"Time out\n";
                return 0;
            }

            curr_state_ptr = m_open.min();
            m_open.pop();
            if( m_closed.count( curr_state_ptr->node_id  ) )
                continue;
            m_closed.insert( curr_state_ptr->node_id );

            NodeIds path;
            extractPath( curr_state_ptr, path );
            std::cout<<m_open.size()<<"\n";
            /*
            if( m_open.size() > 1000 ){
                std::cout<<m_open.size()<<"\n";
                if( !constrained_pspace_ptr->satisfiesConstraints( path ) ){
                    m_suspended.push( curr_state_ptr );
                    continue;
                }
            }
            */

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
        return 0;
    }
}
