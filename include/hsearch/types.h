#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <vector>
#include <utility>
#include <memory>

//#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>

namespace hsearch {
    using Node = size_t;
    using Nodes = std::vector<Node>;

    using NodeId = int;
    using NodeIds = std::vector<NodeId>;

    using Edge = std::pair<Node, Node>;
    using Edges = std::vector<Edge>;

    using Action = std::vector<double>;
    using Actions = std::vector<Action>;

    using OccupancyGrid = smpl::OccupancyGrid;
    using OccupancyGridPtr = std::shared_ptr<smpl::OccupancyGrid>;

    //using CollisionCheckerPtr = std::unique_ptr<smpl::CollisionChecker>;

} //namespace hsearch

#endif
