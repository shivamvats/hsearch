#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <random>

#include <hsearch/types.h>
#include <hsearch/collision_checking/two_dim_grid_collision_checker.h>
#include <hsearch/space/action_space.h>
#include <hsearch/space/constrained_two_dim_grid_space.h>
#include <hsearch/search/constrained_dijkstra.h>
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
    const double size_x = pixel_res_*img_.cols;
    const double size_y = pixel_res_*img_.rows;
    const double size_z = 1.5*pixel_res_;
    double max_dist = 1;
    double origin_x=0, origin_y=0, origin_z=0;
    OccupancyGridPtr occ_grid_ptr = std::make_shared<OccupancyGrid>( size_x, size_y,
                    size_z, pixel_res_, origin_x, origin_y, origin_z, max_dist );

    std::vector<smpl::Vector3> occupied_points;
    smpl::Vector3 v {0, 0, 0};
    for( int i=0; i<img_.rows; i++ ){
        for( int j=0; j<img_.cols; j++ ){
            if( int(img_.at<uchar>( i, j )) < 100 ){
                occ_grid_ptr->gridToWorld( j, i, 0, v[0], v[1], v[2] );
                occupied_points.push_back(v);
            }
        }
    }
    occ_grid_ptr->addPointsToField( occupied_points );
    //cout<<"Occupied voxels: "<<occ_grid_ptr->getOccupiedVoxelCount()<<"\n";
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

bool getRandStartGoal(
        TwoDimGridCollisionCheckerPtr collision_checker_ptr_,
        double size_x,
        double size_y,
        double min_dist_,
        RobotState& start_,
        RobotState& goal_,
        double seed_=100 ){
    std::default_random_engine generator( seed_ );
    std::uniform_real_distribution<double> distribution( 0.0, 1.0 );

    bool pair_found = false;
    while( !pair_found ){
        bool start_found = false;
        double x = distribution(generator)*size_x;
        double y = distribution(generator)*size_y;
        start_ = {x, y};
        if(!collision_checker_ptr_->isStateValid( start_ ))
            continue;
        start_found = true;

        bool goal_found = false;
        x = distribution(generator)*size_x;
        y = distribution(generator)*size_y;
        goal_ = {x, y};
        if( !collision_checker_ptr_->isStateValid( goal_ ) )
            continue;
        goal_found = true;

        auto sq_distance_bw = [](RobotState a, RobotState b){
            return ( a[0] - b[0] )*( a[0] - b[0] ) +
                ( a[1] - b[1] )*( a[1] - b[1] );
        };
        if( start_found && goal_found &&
                sq_distance_bw( start_, goal_ ) > min_dist_*min_dist_ )
            pair_found = true;
    }
    if( pair_found )
        return true;
    else
        return false;

}

void testConstrainedTwoDimGrid( char* img_path_ ){
    cout<<"Testing Constrained Two Dim Grid Space\n";
    cout<<"=======================================\n";

    cv::Mat img = cv::imread( img_path_, CV_LOAD_IMAGE_GRAYSCALE );
    Visualizer viz( img );
    viz.imshow(0);

    cv::Point start( 50, 50 );
    double res = 0.01;
    int connectivity = 4;
    double planning_res = 4*res;

    auto grid_ptr = constructOccGrid( img, res );
    auto collision_checker_ptr = make_shared<TwoDimGridCollisionChecker>( grid_ptr.get() );
    auto action_space_ptr = constructActionSpace( planning_res, connectivity );
    std::vector<double> s = {res*start.x, res*start.y };

    auto pspace_ptr = make_shared<ConstrainedTwoDimGridSpace>(
            collision_checker_ptr,
            action_space_ptr,
            s,
            res );
    pspace_ptr->setSimilarityThresh( 0.1 );

    RobotState rand_start, rand_goal;
    getRandStartGoal( collision_checker_ptr, img.cols*res, img.rows*res, 5*res, rand_start, rand_goal );
    pspace_ptr->setStart( rand_start );
    pspace_ptr->setGoal( rand_goal );
    pspace_ptr->setGoalThresh( 0.1 );

    auto lattice_space = dynamic_pointer_cast<LatticePlanningSpace>(pspace_ptr);
    ConstrainedDijkstra planner( lattice_space );

    //******************************
    //Plan to generate a constraint
    //******************************
    NodeIds soltn;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto solved = planner.plan( 5, soltn ); auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = finish_time - start_time;
    if( !solved ){
        throw "Failed to plan.";
    }
    cout<<"Planning succeeded.\n";
    cout<<"Time taken: "<<elapsed_time.count()<<"s\n";
    cout<<"Expansions: "<<planner.m_closed.size()<<"\n";
    LineString constraint_curve;
    RobotState state;
    for( auto& id: soltn ){
        state = pspace_ptr->nodeIdToRobotState( id );
        constraint_curve.push_back( PointXY( state[0], state[1] ) );
    }

    if( solved ){
        std::vector<RobotCoord> path_points;
        for( auto id: soltn )
            path_points.push_back( pspace_ptr->nodeIdToRobotCoord( id ) );
        visualizeSoltn( viz, path_points );
    }

    //***********************
    //Replan with Constraint
    //***********************

    pspace_ptr->addPathConstraint( constraint_curve );
    soltn.clear();
    pspace_ptr->clear();
    pspace_ptr->setStart( rand_start );
    pspace_ptr->setGoal( rand_goal );
    pspace_ptr->setGoalThresh( 0.1 );

    planner.reinit();

    // Constrained Planning
    start_time = std::chrono::high_resolution_clock::now();
    solved = planner.plan( 5, soltn );
    finish_time = std::chrono::high_resolution_clock::now();
    elapsed_time = finish_time - start_time;
    if( !solved ){
        for( auto& it: planner.m_node_expansions ){
            RobotCoord robot_coord = pspace_ptr->nodeIdToRobotCoord( it );
            viz.markPoint( robot_coord[0], robot_coord[1], 1, std::array<int, 3>{ 100, 100, 100 } );
            //viz.imshow(1);
        }
        viz.imshow(1);
        cout<<"Planning Failed.\n";
        cout<<"Time taken: "<<elapsed_time.count()<<"s\n";
        cout<<"Expansions: "<<planner.m_closed.size()<<"\n";
        throw "Failed to plan.";
    }
    cout<<"Planning succeeded.\n";
    cout<<"Time taken: "<<elapsed_time.count()<<"s\n";
    cout<<"Expansions: "<<planner.m_closed.size()<<"\n";

    if( solved ){
        std::vector<RobotCoord> path_points;
        for( auto id: soltn )
            path_points.push_back( pspace_ptr->nodeIdToRobotCoord( id ) );
        visualizeSoltn( viz, path_points );
    }
}

int main( int argc, char** argv ){
    if( argc != 2 ){
        cout<<"Path to image not found."<<"\n";
        return -1;
    }
    testConstrainedTwoDimGrid( argv[1] );
}
