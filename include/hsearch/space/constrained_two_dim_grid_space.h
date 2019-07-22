#ifndef CONSTRAINED_TWO_DIM_GRID_SPACE
#define CONSTRAINED_TWO_DIM_GRID_SPACE

#include "two_dim_grid_space.h"

namespace hsearch {

    class ConstrainedTwoDimGridSpace : public TwoDimGridSpace {
        public:
        ConstrainedTwoDimGridSpace(
                CollisionCheckerPtr,
                ActionSpacePtr,
                RobotState start_,
                const double res_ );
        ~ConstrainedTwoDimGridSpace();

        bool addPathConstraint( const LineString& );
        bool setSimilarityThresh( const double );
        bool satisfiesConstraints( const NodeIds );

        public:
        std::vector<LineString> m_path_constraints;
        double m_thresh;
    };

}

#endif /* ifndef CONSTRAINED_TWO_DIM_GRID_SPACE

 */
