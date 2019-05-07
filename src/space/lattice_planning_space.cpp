#include "hsearch/space/lattice_planning_space.h"

namespace hsearch {
    LatticePlanningSpace::LatticePlanningSpace(
            ActionSpacePtr &action_space_ptr_,
            RobotState start_ ) :
        // collision_checker,
        m_action_space_ptr( action_space_ptr_ ),
        m_start( start_ ){
            Lattice temp;
            m_lattice = temp;
    }

    RobotStates LatticePlanningSpace::Succs( const RobotState s_ ) const {
        RobotStates succs = m_action_space_ptr->applyActions( s_ );
        return succs;
    }

    size_t LatticePlanningSpace::dim() const {
        return m_action_space_ptr->dim();
    }
}
