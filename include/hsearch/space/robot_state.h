#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <hsearch/types.h>

namespace hsearch {
    class RobotState {
        public:
        RobotState( const std::vector<double> );
        RobotState( const RobotState& );
        RobotState add( const std::vector<double>& ) const;
        RobotState& operator=( const RobotState& );
        size_t size() const;
        void print() const;

        std::vector<double> m_data;
    };

    using RobotStates = std::vector<RobotState>;

}
#endif /* ifndef ROBOT_STATE_H */
