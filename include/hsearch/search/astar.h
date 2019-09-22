#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <unordered_set>
#include <smpl/heap/intrusive_heap.h>
#include "lattice_planner.h"

namespace hsearch{
    class Astar {
        public:
        Astar( LatticePlanningSpacePtr& );
        ~Astar();

        virtual bool isGoal(const NodeId&) const override;
        virtual bool plan( const double, NodeIds& ) override;

        struct SearchState : public smpl::heap_element {
            SearchState* bp;
            enum Flags {
                Closed = (1 << 0),
                Suspended = (1 << 1)
            };

            int node_id;
            int g;
            int h;
            int f;
            short call_number;
        };

        struct SearchStateCompare {
            bool operator()(const SearchState& s1, const SearchState& s2) const {
                return s1.g < s2.g;
            }
        };

        using SearchStatePtr = SearchState*;

        SearchStatePtr getSearchStatePtr(const NodeId&);

        using OpenList = smpl::intrusive_heap<SearchState, SearchStateCompare>;

        OpenList m_open;
        // Only needed to query existence.
        std::unordered_set<NodeId> m_closed;
        std::unordered_map<NodeId, SearchStatePtr> m_nodeid_to_search_state;

        NodeId m_start_node_id;

    };
}

#endif /* ifndef ASTAR_PLANNER_H */
