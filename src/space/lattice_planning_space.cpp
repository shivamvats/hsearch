#include <cmath>
#include <stdexcept>
#include <algorithm>

#include "hsearch/space/lattice_planning_space.h"

namespace hsearch {

    LatticePlanningSpace::LatticePlanningSpace(
            OccupancyGridPtr occ_grid_ptr_,
            ActionSpacePtr action_space_ptr_,
            RobotState start_ ) :
        // collision_checker,
        m_action_space_ptr( action_space_ptr_ ),
        m_start( start_ ),
        m_res( occ_grid_ptr_->resolution() ){}

    RobotStates LatticePlanningSpace::Succs( const RobotState& s_ ) {
        RobotStates succs = m_action_space_ptr->applyActions( s_ );
        return succs;
    }

    NodeIds LatticePlanningSpace::Succs( const NodeId& node_id_ ){
        RobotState robot_state = nodeIdToRobotState( node_id_ );
        auto robot_succs = Succs( robot_state );
        std::vector<NodeId> succs;
        succs.resize( robot_succs.size() );
        std::transform( robot_succs.begin(), robot_succs.end(), succs.begin(), [this]( RobotState s ){
                return robotStateToNodeId( s );
                } );
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

    bool LatticePlanningSpace::setGoal( const RobotState state_ ){
        m_goal = state_;
        return true;
    }

    bool LatticePlanningSpace::setGoalThresh( double thresh_ ){
        m_goal_thresh = thresh_;
        return true;
    }

    bool LatticePlanningSpace::isGoal( const RobotState& state_ ) const {
        for( size_t i=0; i<dim(); i++ ){
            if( fabs( state_[i] - m_goal[i] ) > m_goal_thresh )
                return false;
        }
        return true;
    }

    bool LatticePlanningSpace::isGoal( const NodeId& node_id_ ) const {
        return isGoal( nodeIdToRobotState( node_id_ ) );
    }

    size_t LatticePlanningSpace::dim() const {
        return m_action_space_ptr->dim();
    }

    RobotCoord LatticePlanningSpace::robotStateToRobotCoord( const RobotState& robot_state_ ) const {
        RobotCoord robot_coord;
        int coord;
        for( double el : robot_state_ ){
            coord = std::round(el/m_res);
            robot_coord.push_back( coord );
        }
        return robot_coord;
    }

    RobotState LatticePlanningSpace::robotCoordToRobotState( const RobotCoord& robot_coord_ ) const {
        RobotState robot_state;
        double state;
        for( auto el : robot_coord_ ){
            state = el*m_res;
            robot_state.push_back( state );
        }
        return robot_state;
    }

    /*Also inserts robot coord to the class maps if not already present.
     */
    NodeId LatticePlanningSpace::robotCoordToNodeId( const RobotCoord& robot_coord_ ) {
        if( m_robot_coord_to_node_id.find( robot_coord_ ) != m_robot_coord_to_node_id.end() ){
            return m_robot_coord_to_node_id[ robot_coord_ ];
        }
        else {
            int id = m_robot_coord_to_node_id.size();
            m_robot_coord_to_node_id[ robot_coord_ ] = id;
            m_node_id_to_robot_coord[ id ] = robot_coord_;
            return id;
        }
    }

    NodeId LatticePlanningSpace::robotStateToNodeId( const RobotState& robot_state_ ) {
        RobotCoord robot_coord = robotStateToRobotCoord( robot_state_ );
        return robotCoordToNodeId( robot_coord );
    }

    RobotCoord LatticePlanningSpace::nodeIdToRobotCoord( const NodeId& node_id_ ) const {
        auto iter = m_node_id_to_robot_coord.find( node_id_ );
        if( iter != m_node_id_to_robot_coord.end() )
            return iter->second;
        else {
            throw std::runtime_error( "node-id not in map." );
        }
    }

    RobotState LatticePlanningSpace::nodeIdToRobotState( const NodeId& node_id_ ) const {
        auto robot_coord = nodeIdToRobotCoord( node_id_ );
        return robotCoordToRobotState( robot_coord );
    }

} //namespace hsearch


