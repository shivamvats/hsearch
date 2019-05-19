#include <iostream>

#include <hsearch/types.h>
#include <hsearch/space/action_space.h>
#include <hsearch/space/lattice_planning_space.h>
#include <hsearch/search/dijkstra.h>

using namespace std;
using namespace hsearch;

OccupancyGrid createOccupancyGrid(){
    double size_x=10, size_y=10, size_z=2;
    double res=0.05;
    double origin_x=0, origin_y=0, origin_z=0;
    double max_dist=1;
    return OccupancyGrid( size_x, size_y, size_z,
            res, origin_x, origin_y, origin_z, max_dist );
}

void testDijkstra(){
    cout<<"Testing Dijkstra Planner\n";
    cout<<"=========================\n";
    auto lattice_ptr = make_shared<Lattice>();

    auto grid = std::make_shared<OccupancyGrid>( createOccupancyGrid() );
    ActionSpacePtr action_space_ptr = make_shared<ActionSpace>( 3 );
    Action action1{ 1, 0, 0 };
    Action action2{ 0, 1, 0 };
    Action action3{ -1, 0, 0 };
    Action action4{ 0, -1, 0 };
    action_space_ptr->addAction( action1 );
    action_space_ptr->addAction( action2 );
    action_space_ptr->addAction( action3 );
    action_space_ptr->addAction( action4 );

    RobotState start( std::vector<double>{ 0, 0, 0 } );
    double res = 0.05;
    auto pspace_ptr = make_shared<LatticePlanningSpace>(
            grid, action_space_ptr, start, res );
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

}

int main(){
    testDijkstra();
}
