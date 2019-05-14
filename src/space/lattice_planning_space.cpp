#include <cmath.h>

#include "hsearch/space/lattice_planning_space.h"

namespace hsearch {
    LatticePlanningSpace::LatticePlanningSpace(
            OccupancyGridPtr &occ_grid_ptr_,
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

    bool LatticePlanningSpace::setStart( const RobotState state_ ){
        //if( isValid( start_ ) ){
        //    m_start = start_;
        //    return true;
        //}
        //else{
        //    ROS_ERROR("Start location is invalid.");
        //    return false;
        //}
        m_start = state_;
        return true;
    }

    bool LatticePlanningSpace::setGoal( const RobotState& state_ ){
        m_goal = state_;
        return true;
    }

    bool LatticePlanningSpace::setGoalThresh( double thresh_ ){
        m_goal_thresh = thresh_;
    }

    bool LatticePlanningSpace::isGoal( const RobotState& state_ ) const {
        for( size_t i=0; i<dim(); i++ ){
            if( fabs( state_[i] - m_goal[i] ) > m_goal_thresh )
                return false;
        }
        return true;
    }

    size_t LatticePlanningSpace::dim() const {
        return m_action_space_ptr->dim();
    }

    NodeId LatticePlanningSpace::robotStateToNodeId( RobotState& robot_state_ ){

    }
}
