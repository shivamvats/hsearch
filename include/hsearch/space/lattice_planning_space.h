#ifndef LATTICE_PLANNING_SPACE_H
#define LATTICE_PLANNING_SPACE_H

#include <hsearch/graph/lattice.h>
#include <hsearch/space/robot_state.h>
#include <hsearch/space/action_space.h>
#include <hsearch/types.h>

namespace hsearch {
    class LatticePlanningSpace {
        public:
        LatticePlanningSpace(
                OccupancyGridPtr&,
                //CollisionCheckerPtr,
                ActionSpacePtr&,
                RobotState );
        virtual RobotStates Succs( const RobotState& ) const;
        //virtual RobotStates Succs( const RobotState, std::vector<bool> )
        bool setStart( const RobotState& );
        bool setGoal( const RobotState& );
        bool setGoalThresh( double );
        bool isGoal( const RobotState& ) const;
        size_t dim() const;

        NodeId robotStateToNodeId( const RobotState& ) const;

        const ActionSpacePtr m_action_space_ptr;
        RobotState m_start;
        RobotState m_goal;
        double m_goal_thresh;
    };
}
#endif /* ifndef LATTICE_PLANNING_SPACE_H */
