#ifndef TWO_DIM_GRID_SPACE
#define TWO_DIM_GRID_SPACE

namespace hsearch {

    class TwoDimGridSpace : public LatticePlanningSpace {
        public:
        TwoDimGridSpace(
                OccupancyGridPtr,
                ActionSpacePtr,
                RobotState start_ );
        // Initialize from an opencv image
        TwoDimGridSpace(
                cv::Mat& img_,
                const int connectivity_,
                const cv::Point& start_ );

        public:
        int m_connectivity;
    };

}

#endif /* ifndef TWO_DIM_GRID_SPACE

 */
