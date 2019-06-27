#include <hsearch/collision_checking/two_dim_grid_collision_checker.h>

namespace hsearch {

    TwoDimGridCollisionChecker::TwoDimGridCollisionChecker( OccupancyGrid* grid_ ) : 
            smpl::Extension(),
            m_grid( grid_ ) {}

    smpl::Extension* TwoDimGridCollisionChecker::getExtension( size_t class_code_ ){
        if ( class_code_ == smpl::GetClassCode<CollisionChecker>() ) {
            return this;
        }
        return nullptr;
    }

    bool TwoDimGridCollisionChecker::isStateValid(
            const RobotState& state_,
            bool verbose_ ) {
        if ( state_.size() < 2 ) {
            std::cout<<"State contains insufficient data";
            return false;
        }
        double x = state_[0];
        double y = state_[1];
        double z = 0.0;
        if ( !m_grid->isInBounds(x, y, z) ) {
            return false;
        }
        if ( m_grid->getDistanceFromPoint(x, y, z) <= 0.0 ) {
            return false;
        }
        return true;
    }

    bool TwoDimGridCollisionChecker::isStateToStateValid(
            const RobotState& start_,
            const RobotState& finish_,
            bool verbose_ ) {
        RobotStates path;
        if ( !interpolatePath(start_, finish_, path) ) {
            return false;
        }
        return std::all_of(
            path.begin(), path.end(),
            [&]( const RobotState& state ){
                return isStateValid(state, false);
            });
    }

    bool TwoDimGridCollisionChecker::interpolatePath(
        const RobotState& start_,
        const RobotState& finish_,
        RobotStates& path_ ){
        //m_grid->resolution();
        const Eigen::Vector2d vstart( start_[0], start_[1] );
        const Eigen::Vector2d vfinish( finish_[0], finish_[1] );
        int num_waypoints =
                (int)std::ceil( (vfinish - vstart).norm() / m_grid->resolution() );
        num_waypoints = std::max(num_waypoints, 2);
        for (int i = 0; i < num_waypoints; ++i) {
            const double alpha = (double)i / (double)(num_waypoints - 1);
            Eigen::Vector2d vinterm = (1.0 - alpha) * vstart + alpha * vfinish;
            RobotState istate(2);
            istate[0] = vinterm.x();
            istate[1] = vinterm.y();
            path_.push_back(std::move(istate));
        }
        return true;
    }
}
