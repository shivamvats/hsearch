#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <hsearch/types.h>
#include <hsearch/collision_checking/two_dim_grid_collision_checker.h>
#include <hsearch/space/action_space.h>
#include <hsearch/space/two_dim_grid_space.h>
#include <hsearch/search/dijkstra.h>
#include <hsearch/viz/visualizer.h>


using namespace std;
using namespace hsearch;

void visualizeSoltn( Visualizer& viz, std::vector<RobotCoord> path_ ){
    for( auto& point: path_ ){
        viz.markPoint( point[0], point[1], 1, std::array<int, 3>{ 100, 100, 100 } );
        viz.imshow(1);
    }
    viz.imshow();
}

OccupancyGridPtr constructOccGrid(
        const cv::Mat& img_,
        const double pixel_res_ ){
    int size_x = pixel_res_*img_.cols;
    int size_y = pixel_res_*img_.rows;
    int size_z = pixel_res_;
    double max_dist = 1;
    double origin_x=0, origin_y=0, origin_z=0;
    OccupancyGridPtr occ_grid_ptr = std::make_shared<OccupancyGrid>( size_x, size_y,
                    size_z, pixel_res_, origin_x, origin_y, origin_z, max_dist );
    return occ_grid_ptr;
}

ActionSpacePtr constructActionSpace(
        const double pixel_res_,
        const int connectivity_ ){
    ActionSpacePtr action_space_ptr = std::make_shared<ActionSpace>( 2 );
    Action action1{ pixel_res_, 0 };
    Action action2{ -pixel_res_, 0 };
    Action action3{ 0, pixel_res_ };
    Action action4{ 0, -pixel_res_ };
    action_space_ptr->addAction( action1 );
    action_space_ptr->addAction( action2 );
    action_space_ptr->addAction( action3 );
    action_space_ptr->addAction( action4 );
    if( connectivity_ == 8 ){
        Action action5{ pixel_res_, pixel_res_ };
        Action action6{ pixel_res_, -pixel_res_ };
        Action action7{ -pixel_res_, pixel_res_ };
        Action action8{ -pixel_res_, -pixel_res_ };
        action_space_ptr->addAction( action5 );
        action_space_ptr->addAction( action6 );
        action_space_ptr->addAction( action7 );
        action_space_ptr->addAction( action8 );
    }
    else if( connectivity_ != 4 ){
        throw std::invalid_argument(" Received invalid value for the connectivity parameter.");
    }
    return action_space_ptr;
}

void testTwoDimGrid( char* img_path_ ){
    cout<<"Testing Two Dim Grid Space\n";
    cout<<"=========================\n";

    cv::Mat img = cv::imread( img_path_, CV_LOAD_IMAGE_GRAYSCALE );
    Visualizer viz( img );
    viz.imshow();

    cv::Point start( 100, 100 );
    double res = 0.1;
    int connectivity = 4;

    auto grid_ptr = constructOccGrid( img, res );
    auto collision_checker_ptr = make_shared<TwoDimGridCollisionChecker>( grid_ptr.get() );
    auto action_space_ptr = constructActionSpace( res, connectivity );
    std::vector<double> s = {res*start.x, res*start.y };

    auto pspace_ptr = make_shared<TwoDimGridSpace>(
            collision_checker_ptr,
            action_space_ptr,
            s,
            res );
    cv::Point goal( 400, 100 );
    pspace_ptr->setGoal( goal );
    pspace_ptr->setGoalThresh( 0.1 );

    //cout<<"Start:\n";
    //printVector( start );
    //cout<<"Goal:\n";
    //printVector( goal );
    auto lattice_space = dynamic_pointer_cast<LatticePlanningSpace>(pspace_ptr);
    //LatticePlanningSpacePtr lattice_space( pspace_ptr.get() );
    Dijkstra planner( lattice_space );

    NodeIds soltn;
    auto solved = planner.plan( 5, soltn );
    if( solved ){
        std::vector<RobotCoord> path_points;
        for( auto id: soltn )
            path_points.push_back( pspace_ptr->nodeIdToRobotCoord( id ) );
        visualizeSoltn( viz, path_points );

        cout<<"Planning succeeded.\n";
    }
    else {
        cout<<"Failed to plan.\n";
    }
}

int main( int argc, char** argv ){
    if( argc != 2 ){
        cout<<"Path to image not found."<<"\n";
        return -1;
    }
    testTwoDimGrid( argv[1] );
}
