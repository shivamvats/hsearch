#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <vector>
#include <utility>
#include <memory>

//#include <smpl/collision_checker.h>

namespace hsearch {
    using Node = size_t;
    using Nodes = std::vector<Node>;

    using Edge = std::pair<Node, Node>;
    using Edges = std::vector<Edge>;

    using Action = std::vector<double>;
    using Actions = std::vector<Action>;

    //using CollisionCheckerPtr = std::unique_ptr<smpl::CollisionChecker>;

} //namespace hsearch

#endif
