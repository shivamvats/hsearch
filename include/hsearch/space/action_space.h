#ifndef ACTION_SPACE_H
#define ACTION_SPACE_H

#include <hsearch/types.h>

namespace hsearch {
    /**
     * The Action space contains a set of actions.
     * The fundamental property of the Action Space is its dimensionality.
     * The Action Space may contain multiple actions, but their size must match
     * the dimensionality of the space.
     */
    class ActionSpace {
        public:
        ActionSpace( int );

        bool addAction( const Action );
        RobotStates applyActions( const RobotState );
        RobotState applyAction( const RobotState, int );
        int dim() const;

        Actions m_actions;
        int m_dim;
    };

    using ActionSpacePtr = std::shared_ptr<ActionSpace>;
} // namespace hsearch

#endif /* ifndef ACTION_SPACE_H */
