#ifndef TWO_DIM_GRID_SPACE
#define TWO_DIM_GRID_SPACE

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "lattice_planning_space.h"

namespace hsearch {

    class TwoDimGridSpace : public LatticePlanningSpace {
        public:
        using LatticePlanningSpace::setGoal;

        TwoDimGridSpace(
                CollisionCheckerPtr,
                ActionSpacePtr,
                RobotState start_,
                const double res_ );

        bool setGoal( const cv::Point );
    };

}

#endif /* ifndef TWO_DIM_GRID_SPACE

 */
