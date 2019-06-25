#include <hsearch/space/two_dim_grid_space.h>

namespace hsearch {

    TwoDimGridSpace::TwoDimGridSpace(
            OccupancyGridPtr occ_grid_ptr_,
            ActionSpacePtr action_space_ptr_,
            RobotState start_ ) :
        LatticePlanningSpace( occ_grid_ptr_, action_space_ptr_, start_ )
    {}

    TwoDimGridSpace::TwoDimGridSpace( 
            cv::Mat& img_,
            const int connectivity_,
            const int pixel_res_,
            const cv::Point& start_ ):
            m_connectivity( connectivity_ ){
        // Occupancy Grid
        int size_x = pixel_res_*img_.cols;
        int size_y = pixel_res_*img_.rows;
        int size_z = pixel_res_;
        double max_dist = 1;
        double origin_x=0, origin_y=0, origin_z=0;
        occ_grid_ptr = std::make_shared<OccupancyGrid>( size_x, size_y,
                       size_z, pixel_res_, origin_x, origin_y, origin_z, max_dist );

        // Action Space
        ActionSpacePtr action_space_ptr = make_shared<Action>( 2 );
        Action action1{ pixel_res_, 0 };
        Action action2{ -pixel_res_, 0 };
        Action action3{ 0, pixel_res_ };
        Action action4{ 0, -pixel_res_ };
        action_space_ptr->addAction( action_1 );
        action_space_ptr->addAction( action_2 );
        action_space_ptr->addAction( action_3 );
        action_space_ptr->addAction( action_4 );
        if( m_connectivity == 8 ){
            Action action5{ pixel_res_, pixel_res_ };
            Action action6{ pixel_res_, -pixel_res_ };
            Action action7{ -pixel_res_, pixel_res_ };
            Action action8{ -pixel_res_, -pixel_res_ };
            action_space_ptr->addAction( action_5 );
            action_space_ptr->addAction( action_6 );
            action_space_ptr->addAction( action_7 );
            action_space_ptr->addAction( action_8 );
        }
        else if( m_connectivity != 4 )
            throw std::invalid_argument(" Received invalid value for the connectivity parameter.");

        std::vector<double> robot_start = {pixel_res_*start_.x, pixel_res_*start_.y };
        LatticePlanningSpace( occ_grid_ptr, action_space_ptr, robot_start );
    }

}
