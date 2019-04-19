#include "types.h"

namespace hsearch {
    /**
     * The Action space contains a set of actions. Each action acts on the full
     * state (it may leave some dimensions unchanged).
     */
    class ActionSpace {
        public:
        ActionSpace();

        bool addAction(std::vector<double> delta);

        std::vector<Action> m_actions;
    };
} // namespace hsearch
