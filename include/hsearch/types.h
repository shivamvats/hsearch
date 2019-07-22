#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <vector>
#include <array>
#include <utility>
#include <memory>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

//#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/collision_checker.h>

namespace hsearch {
    using Node = size_t;
    using Nodes = std::vector<Node>;

    using NodeId = int;
    using NodeIds = std::vector<NodeId>;

    using RobotState = std::vector<double>;
    using RobotStates = std::vector<RobotState>;

    using RobotCoord = std::vector<int>;
    using RobotCoords = std::vector<RobotCoord>;

    using Edge = std::pair<Node, Node>;
    using Edges = std::vector<Edge>;

    using Action = std::vector<double>;
    using Actions = std::vector<Action>;

    using OccupancyGrid = smpl::OccupancyGrid;
    using OccupancyGridPtr = std::shared_ptr<smpl::OccupancyGrid>;

    using CollisionChecker = smpl::CollisionChecker;
    using CollisionCheckerPtr = std::shared_ptr<CollisionChecker>;

    using Color = std::array<int, 3>;

    using PointXY = boost::geometry::model::d2::point_xy<double>;
    using LineString = boost::geometry::model::linestring<PointXY>;
    //using CollisionCheckerPtr = std::unique_ptr<smpl::CollisionChecker>;

    inline RobotState addRobotStates( RobotState& a_, RobotState& b_ ){
        assert( a_.size() == b_.size() );

        RobotState sum;
        for( size_t i=0; i<a_.size(); i++)
            sum.push_back( a_[i] + b_[i] );
        return sum;
    }

    template <typename T>
    inline void printVector( const std::vector<T> state_ ){
        for( size_t i=0; i<state_.size(); i++ )
            std::cout<<state_[i]<<"  ";
        std::cout<<"\n";
    }

} //namespace hsearch

#endif
