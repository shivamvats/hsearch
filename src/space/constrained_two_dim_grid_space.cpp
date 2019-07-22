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
        if( N > m_path_constraints[0].size() )
            return true;
        auto path_constraint = LineString( m_path_constraints[0].begin(), m_path_constraints[0].begin() + N );

        LineString path;
        RobotState state;
        for( auto& id: node_ids_ ){
            state = nodeIdToRobotState( id );
            path.push_back( PointXY( state[0], state[1] ) );
        }
        double dist = boost::geometry::discrete_frechet_distance(
                path, path_constraint );
        //std::cout<<dist<<"\n";
        return dist > (double(m_thresh*N))/path_constraint.size();
    }

}

