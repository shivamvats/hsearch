#include <assert.h>
#include "hsearch/types.h"
#include "hsearch/space/robot_state.h"

namespace hsearch {
    RobotState::RobotState( std::vector<double> data_ ){
        m_data = data_;
        m_size = data_.size();
    }

    RobotState RobotState::add( std::vector<double> addendum_ ){
        assert( size() == addendum_.size() );

        std::vector<double> sum;
        for ( int i=0; i<m_size; i++ )
            sum.push_back( m_data[i] + addendum_[i] );

        return RobotState( sum );
    }

    size_t RobotState::size() const {
        return m_size;
    }

    void RobotState::print() const {
        for( int i=0; i<m_size; i++ )
            std::cout<<m_data[i]<<"\t";
        std::cout<<"\n";
    }


}
