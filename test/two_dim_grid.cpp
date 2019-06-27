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

void visualizeSoltn( Visualizer& viz, std::vector<RobotCoord> path_ ){
    for( auto& point: path_ ){
        viz.markPoint( point[0], point[1], 1, std::array<int, 3>{ 100, 100, 100 } );
        viz.imshow(1);
    }
    viz.imshow();
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
    auto pspace_ptr = make_shared<TwoDimGridSpace>(
            img, connectivity, res, start );
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
