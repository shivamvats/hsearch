#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <unordered_set>
#include <smpl/heap/intrusive_heap.h>
#include "lattice_planner.h"

namespace hsearch {
    class Dijkstra : public LatticePlanner {
        public:
        Dijkstra( LatticePlanningSpacePtr& );
        ~Dijkstra();

        virtual bool isGoal( const NodeId& ) const override;
        virtual bool plan( const double, NodeIds& ) override;

        struct SearchState : public smpl::heap_element {
            SearchState* bp;
            //enum Flags {
            //    Closed = (1 << 0),
            //    Suspended = (1 << 1),
            //};

            int node_id;
            int g;     // cost-to-come
            //int h;     // estimated cost-to-go
            //int f;     // (g + eps * h) at time of insertion into OPEN
            //int level;
            short call_number;
            //std::uint8_t flags;
        };

        struct SearchStateCompare {
            bool operator()(const SearchState& s1, const SearchState& s2) const {
                return s1.g < s2.g;
            }
        };
        using SearchStatePtr = SearchState*;

        SearchStatePtr getSearchStatePtr( const NodeId& );
        bool extractPath( SearchStatePtr goal_, NodeIds& path_ );
        virtual void clear();

        using OpenList = smpl::intrusive_heap<SearchState, SearchStateCompare>;

        OpenList m_open;
        std::unordered_set<NodeId> m_closed;
        std::unordered_map<NodeId, SearchStatePtr> m_node_id_to_search_state;

        NodeId m_start_node_id;

    };
}

#endif
