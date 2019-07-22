#include <boost/geometry.hpp>

#include <hsearch/space/constrained_two_dim_grid_space.h>

namespace hsearch {

    ConstrainedTwoDimGridSpace::ConstrainedTwoDimGridSpace(
            CollisionCheckerPtr collision_checker_,
            ActionSpacePtr action_space_ptr_,
            RobotState start_,
            const double res_ ) :
        TwoDimGridSpace( collision_checker_, action_space_ptr_, start_, res_ )
    {}

    bool ConstrainedTwoDimGridSpace::addPathConstraint( const LineString& line_string_ ){
        m_path_constraints.push_back( line_string_ );
    }

    bool ConstrainedTwoDimGridSpace::setSimilarityThresh( const double thresh_ ){
        m_thresh = thresh_;
    }

    bool ConstrainedTwoDimGridSpace::satisfiesConstraints( const NodeIds node_ids_ ){
        LineString path;
        RobotState state;
        for( auto& id: node_ids_ ){
            state = nodeIdToRobotState( id );
            path.push_back( PointXY( state[0], state[1] ) );
        }
        return boost::geometry::discrete_frechet_distance(
                path, m_path_constraints[0] ) > m_thresh;
    }

}

