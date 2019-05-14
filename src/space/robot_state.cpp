#include <assert.h>
#include "hsearch/types.h"
#include "hsearch/space/robot_state.h"

namespace hsearch {
    RobotState::RobotState( const std::vector<double> data_ ){
        m_data = data_;
    }

    RobotState::RobotState( const RobotState& src_ ){
        m_data = src_.m_data;
    }

    RobotState RobotState::add( const std::vector<double>& addendum_ ) const {
        assert( size() == addendum_.size() );

        std::vector<double> sum;
        for ( int i=0; i<size(); i++ )
            sum.push_back( m_data[i] + addendum_[i] );

        return RobotState( sum );
    }

    RobotState& RobotState::operator=(const RobotState& state_ ){
        if( this == &state_ )
            return *this;
        m_data = state_.m_data;
        return *this;
    }

    size_t RobotState::size() const {
        return m_data.size();
    }

    void RobotState::print() const {
        for( int i=0; i<size(); i++ )
            std::cout<<m_data[i]<<"\t";
        std::cout<<"\n";
    }


}
