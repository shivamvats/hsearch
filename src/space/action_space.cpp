#include "hsearch/space/action_space.h"

namespace hsearch {
    ActionSpace::ActionSpace( size_t dim_ ){
        m_dim = dim_;
    }

    bool ActionSpace::addAction( Action action_ ){
        assert( action_.size() == m_dim);
        m_actions.push_back( action_ );
    }

    RobotStates ActionSpace::applyActions( RobotState s_ ){
        assert( s_.size() == m_dim );

        RobotStates succs;
        //for ( auto a : m_actions ){
        //    succs.push_back( s_.add( a ) );
        //}
        for( int i=0; i<m_actions.size(); i++ )
            succs.push_back( applyAction( s_, i ) );
        return succs;
    }

    RobotState ActionSpace::applyAction( RobotState s_, size_t i ){
        assert( s_.size() == m_dim );
        assert( i < m_actions.size() );

        return RobotState( s_.add( m_actions[i] ) );
    }

    size_t ActionSpace::dim() const {
        return m_dim;
    }
}
