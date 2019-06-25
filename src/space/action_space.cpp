#include "hsearch/space/action_space.h"

namespace hsearch {
    ActionSpace::ActionSpace( int dim_ ){
        m_dim = dim_;
    }

    bool ActionSpace::addAction( Action action_ ){
        assert( action_.size() == m_dim);
        m_actions.push_back( action_ );
    }

    RobotStates ActionSpace::applyActions( RobotState s_ ){
        assert( s_.size() == m_dim );

        RobotStates succs;
        for( int i=0; i<m_actions.size(); i++ )
            succs.push_back( applyAction( s_, i ) );
        return succs;
    }

    RobotState ActionSpace::applyAction( RobotState s_, int i ){
        assert( s_.size() == m_dim );
        assert( i < m_actions.size() );

        return addRobotStates( s_, m_actions[i] );
    }

    int ActionSpace::dim() const {
        return m_dim;
    }
}
