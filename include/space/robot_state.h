#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "types.h"

namespace hsearch {
    class RobotState {
        public:
        RobotState( std::vector<double> );
        RobotState add( std::vector<double> );
        size_t size() const;
        void print() const;

        std::vector<double> m_data;
        size_t m_size;
    };

    using RobotStates = std::vector<RobotState>;

}
#endif /* ifndef ROBOT_STATE_H */
