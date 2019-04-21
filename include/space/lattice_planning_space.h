#ifndef LATTICE_PLANNING_SPACE_H
#define LATTICE_PLANNING_SPACE_H

#include "graph/lattice.h"
#include "space/robot_state.h"
#include "space/action_space.h"
#include "types.h"

namespace hsearch {
    class LatticePlanningSpace {
        public:
        LatticePlanningSpace(
                //CollisionCheckerPtr,
                ActionSpacePtr&,
                RobotState );
        virtual RobotStates Succs( const RobotState ) const;
        //virtual RobotStates Succs( const RobotState, std::vector<bool> )
        size_t dim() const;

        const ActionSpacePtr m_action_space_ptr;
        const RobotState m_start;
        Lattice m_lattice;
    };
}
#endif /* ifndef LATTICE_PLANNING_SPACE_H */
