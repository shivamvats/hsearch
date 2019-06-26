#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <hsearch/types.h>
#include <hsearch/space/two_dim_grid_space.h>
#include <hsearch/search/dijkstra.h>
#include <hsearch/viz/visualizer.h>

using namespace std;
using namespace hsearch;

void testTwoDimGrid( char* img_path_ ){
    cout<<"Testing Two Dim Grid Space\n";
    cout<<"=========================\n";

    cv::Mat img = cv::imread( img_path_, CV_LOAD_IMAGE_GRAYSCALE );
    Visualizer viz( img );
    viz.imshow();


    /*
    RobotState start( std::vector<double>{ 0, 0, 0 } );
    double res = 0.05;
    auto pspace_ptr = make_shared<TwoDimGridSpace>(
            grid, action_space_ptr, start );
    RobotState goal = {5, 5, 0};
    pspace_ptr->setGoal( goal );
    pspace_ptr->setGoalThresh( 0.1 );

    cout<<"Start:\n";
    printVector( start );
    cout<<"Goal:\n";
    printVector( goal );
    Dijkstra planner( pspace_ptr );

    NodeIds soltn;
    auto solved = planner.plan( 5, soltn );
    if( solved )
        cout<<"Failed to plan.\n";
    else {
        cout<<"Planning succeeded.\n";
    }
    */

}

int main( int argc, char** argv ){
    if( argc != 2 ){
        cout<<"Path to image not found."<<"\n";
        return -1;
    }
    testTwoDimGrid( argv[1] );
}
