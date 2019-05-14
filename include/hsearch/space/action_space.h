#ifndef ACTION_SPACE_H
#define ACTION_SPACE_H

#include "robot_state.h"
#include "hsearch/types.h"

namespace hsearch {
    /**
     * The Action space contains a set of actions.
     * The fundamental property of the Action Space is its dimensionality.
     * The Action Space may contain multiple actions, but their size must match
     * the dimensionality of the space.
     */
    class ActionSpace {
        public:
        ActionSpace( size_t );

        bool addAction( Action );
        RobotStates applyActions( RobotState );
        RobotState applyAction( RobotState, size_t );
        size_t dim() const;

        Actions m_actions;
        size_t m_dim;
    };

    using ActionSpacePtr = std::shared_ptr<ActionSpace>;
} // namespace hsearch

#endif /* ifndef ACTION_SPACE_H */
