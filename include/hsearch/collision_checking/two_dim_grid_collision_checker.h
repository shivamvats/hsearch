#ifndef TWO_DIM_GRID_COLLISION_CHECKER
#define TWO_DIM_GRID_COLLISION_CHECKER

#include <hsearch/types.h>

namespace hsearch {

    class TwoDimGridCollisionChecker : public CollisionChecker {
        public:
        TwoDimGridCollisionChecker( OccupancyGrid* );
        smpl::Extension* getExtension( size_t class_code_ ) override;

        bool isStateValid(
                const RobotState&,
                bool verbose_=false ) override;

        bool isStateToStateValid(
            const RobotState&,
            const RobotState&,
            bool verbose_ ) override;

        bool interpolatePath(
            const RobotState&,
            const RobotState&,
            RobotStates& path_ ) override;

        public:
        OccupancyGrid* m_grid;
    };

    using TwoDimGridCollisionCheckerPtr = std::shared_ptr<TwoDimGridCollisionChecker>;

}

#endif /* ifndef TWO_DIM_COLLISION_CHECKER */
