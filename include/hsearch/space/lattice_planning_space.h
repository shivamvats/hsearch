#ifndef LATTICE_PLANNING_SPACE_H
#define LATTICE_PLANNING_SPACE_H

#include <unordered_map>
#include <boost/functional/hash.hpp>

#include <hsearch/graph/lattice.h>
#include <hsearch/space/action_space.h>
#include <hsearch/types.h>

namespace hsearch {

    struct RobotCoordHash {
        size_t operator()( const RobotCoord& robot_coord_ ) const {
        return boost::hash_range( robot_coord_.begin(), robot_coord_.end() );
        }
    };

    class LatticePlanningSpace {
        public:
        LatticePlanningSpace(
                CollisionCheckerPtr,
                ActionSpacePtr,
                RobotState start_,
                const double res_ );
        virtual RobotStates Succs( const RobotState& );
        virtual NodeIds Succs( const NodeId& );
        //virtual RobotStates Succs( const RobotState, std::vector<bool> )
        bool setStart( const RobotState );
        bool setGoal( const RobotState );
        bool setGoalThresh( double );
        bool isGoal( const RobotState& ) const;
        bool isGoal( const NodeId& ) const;
        size_t dim() const;

        RobotCoord robotStateToRobotCoord( const RobotState& ) const;
        RobotState robotCoordToRobotState( const RobotCoord& ) const;
        NodeId robotCoordToNodeId( const RobotCoord& );
        NodeId robotStateToNodeId( const RobotState& );
        RobotCoord nodeIdToRobotCoord( const NodeId&  ) const;
        RobotState nodeIdToRobotState( const NodeId& ) const;

        void clear();

        public:
        CollisionCheckerPtr m_collision_checker_ptr;
        const ActionSpacePtr m_action_space_ptr;
        double m_res;
        RobotState m_start;
        RobotState m_goal;
        double m_goal_thresh;

        std::unordered_map<RobotCoord, NodeId, RobotCoordHash> m_robot_coord_to_node_id;
        std::unordered_map<NodeId, RobotCoord> m_node_id_to_robot_coord;
    };

    using LatticePlanningSpacePtr = std::shared_ptr<LatticePlanningSpace>;
}
#endif /* ifndef LATTICE_PLANNING_SPACE_H */
