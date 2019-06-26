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
                OccupancyGridPtr,
                ActionSpacePtr,
                RobotState start_ );
        // Initialize from an opencv image
        TwoDimGridSpace(
                const cv::Mat& img_,
                const int connectivity_,
                const double pixel_res_,
                const cv::Point& start_ );

        bool setGoal( const cv::Point );

        // Helper functions
        OccupancyGridPtr constructOccGrid( const cv::Mat&, const double pixel_res_ );
        ActionSpacePtr constructActionSpace( const double pixel_res_, const int connectivity_ );


        public:
        int m_connectivity;
    };

}

#endif /* ifndef TWO_DIM_GRID_SPACE

 */
