#include <hsearch/space/two_dim_grid_space.h>

namespace hsearch {

    TwoDimGridSpace::TwoDimGridSpace(
            CollisionCheckerPtr collision_checker_,
            ActionSpacePtr action_space_ptr_,
            RobotState start_,
            const double res_ ) :
        LatticePlanningSpace( collision_checker_, action_space_ptr_, start_, res_ )
    {}

    bool TwoDimGridSpace::setGoal( const cv::Point goal_ ){
        RobotState goal = { m_res*goal_.x, m_res*goal_.y };
        return setGoal( goal );
    }

}

