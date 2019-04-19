#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <vector>
#include <utility>

namespace hsearch {
    using Node = size_t;
    using Nodes = std::vector<size_t>;

    using Edge = std::pair<Node, Node>;
    using Edges = std::vector<Edge>;

    using Action = std::vector<double>;
    using Actions = std::vector<Action>;

} //namespace hsearch

#endif
