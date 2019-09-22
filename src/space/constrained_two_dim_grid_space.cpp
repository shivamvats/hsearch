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
        if( m_path_constraints.empty() )
            return true;

        int N = node_ids_.size();
        for( auto& constraint: m_path_constraints ){
            if( N > constraint.size() )
                continue;
            auto path_constraint = LineString( constraint.begin(), constraint.begin() + N );

            LineString path;
            RobotState state;
            for( auto& id: node_ids_ ){
                state = nodeIdToRobotState( id );
                path.push_back( PointXY( state[0], state[1] ) );
            }
            double dist = boost::geometry::discrete_frechet_distance(
                    path, path_constraint );
            //double dist = boost::geometry::discrete_hausdorff_distance(
            //        path, path_constraint );
            if( dist < (double(m_thresh*N))/path_constraint.size() )
                return false;
        }
        return true;
    }

}

